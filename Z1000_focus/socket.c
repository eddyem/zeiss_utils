/*
 *                                                                                                  geany_encoding=koi8-r
 * socket.c - socket IO
 *
 * Copyright 2018 Edward V. Emelianov <eddy@sao.ru, edward.emelianoff@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */
#include "can_encoder.h"
#include "HW_dependent.h"
#include "usefull_macros.h"
#include "socket.h"
#include <netdb.h>      // addrinfo
#include <arpa/inet.h>  // inet_ntop
#include <pthread.h>
#include <limits.h>     // INT_xxx
#include <math.h>   // fabs
#include <signal.h> // pthread_kill
#include <unistd.h> // daemon
#include <sys/syscall.h> // syscall

#include "cmdlnopts.h"   // glob_pars

#define BUFLEN    (10240)
// Max amount of connections
#define BACKLOG   (30)

extern glob_pars *G;

/**************** COMMON FUNCTIONS ****************/
/**
 * wait for answer from socket
 * @param sock - socket fd
 * @return 0 in case of error or timeout, 1 in case of socket ready
 */
static int waittoread(int sock){
    fd_set fds;
    struct timeval timeout;
    int rc;
    timeout.tv_sec = 1; // wait not more than 1 second
    timeout.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    do{
        rc = select(sock+1, &fds, NULL, NULL, &timeout);
        if(rc < 0){
            if(errno != EINTR){
                WARN("select()");
                return 0;
            }
            continue;
        }
        break;
    }while(1);
    if(FD_ISSET(sock, &fds)) return 1;
    return 0;
}

/**************** SERVER FUNCTIONS ****************/
// `canbus_mutex` used to exclude simultaneous CAN messages
// `moving_mutex` used to block simultaneous attempts to move motor
static pthread_mutex_t canbus_mutex = PTHREAD_MUTEX_INITIALIZER, moving_mutex = PTHREAD_MUTEX_INITIALIZER;
bool emerg_stop = FALSE;

/**
 * Send data over socket
 * @param sock      - socket fd
 * @param webquery  - ==1 if this is web query
 * @param buf - buffer with data (zero-terminated)
 * @return 1 if all OK
 */
static int send_data(int sock, int webquery, char *buf){
    if(!buf) return 0;
    ssize_t L, Len = strlen(buf);
    //DBG("buf: %s, Len: %zd", buf, Len);
    if(Len < 1) return 0;
    char tbuf[BUFLEN];
    // OK buffer ready, prepare to send it
    if(webquery){
        L = snprintf(tbuf, BUFLEN,
            "HTTP/2.0 200 OK\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST\r\n"
            "Access-Control-Allow-Credentials: true\r\n"
            "Content-type: text/plain\r\nContent-Length: %zd\r\n\r\n", Len);
        if(L < 0){
            WARN("sprintf()");
            return 0;
        }
        if(L != write(sock, tbuf, L)){
            WARN("write");
            return 0;
        }
    }
    if(Len != write(sock, buf, Len)){
        WARN("write()");
        return 0;
    }
    return 1;
}

// search a first word after needle without spaces
static char* stringscan(char *str, char *needle){
    char *a, *e;
    char *end = str + strlen(str);
    a = strstr(str, needle);
    if(!a) return NULL;
    a += strlen(needle);
    while (a < end && (*a == ' ' || *a == '\r' || *a == '\t' || *a == '\r')) a++;
    if(a >= end) return NULL;
    e = strchr(a, ' ');
    if(e) *e = 0;
    return a;
}

/**
 * @brief move_focus - separate thread moving focus to given position
 * @param targpos - target position
 */
static void *move_focus(void *targpos){
    double pos = *((double*)targpos);
    DBG("MOVE FOCUS: %g", pos);
    pthread_mutex_lock(&canbus_mutex);
    // in any error case we should check end-switches and move out of them!
    if(move2pos(pos)) go_out_from_ESW();
    pthread_mutex_unlock(&moving_mutex);
    pthread_mutex_unlock(&canbus_mutex);
    pthread_exit(NULL);
    return NULL;
}

static const char *startmoving(double pos){
    static double sp;
    if(pthread_mutex_trylock(&moving_mutex)) return S_ANS_MOVING;
    pthread_t m_thread;
    DBG("startmoving: %g", pos);
    sp = pos;
    if(pthread_create(&m_thread, NULL, move_focus, (void*) &sp)){
        WARN("pthread_create()");
        pthread_mutex_unlock(&moving_mutex);
        return S_ANS_ERR;
    }else{
        DBG("Thread created, detouch");
        pthread_detach(m_thread); // don't care about thread state
    }
    return S_ANS_OK;
}

/**
 * @brief ego, getoutESW - run go_out_from_ESW in a separate thread
 */
static void *ego(_U_ void *unused){
    DBG("MOVE OUT FROM END-SWITCH");
    pthread_mutex_lock(&canbus_mutex);
    go_out_from_ESW();
    pthread_mutex_unlock(&canbus_mutex);
    pthread_mutex_unlock(&moving_mutex);
    pthread_exit(NULL);
    return NULL;
}
static void getoutESW(){
    pthread_mutex_lock(&moving_mutex);
    pthread_t m_thread;
    if(pthread_create(&m_thread, NULL, ego, NULL)){
        WARN("pthread_create()");
        pthread_mutex_unlock(&moving_mutex);
    }else{
        DBG("Thread created, detouch");
        pthread_detach(m_thread); // don't care about thread state
    }
}

static void *handle_socket(void *asock){
#define getparam(x)     (strncmp(found, x, sizeof(x)-1) == 0)
    //putlog("handle_socket(): getpid: %d, pthread_self: %lu, tid: %lu",getpid(), pthread_self(), syscall(SYS_gettid));
    int sock = *((int*)asock);
    int webquery = 0; // whether query is web or regular
    char buff[BUFLEN];
    ssize_t rd;
    double t0 = dtime();
    while(dtime() - t0 < SOCKET_TIMEOUT){
        if(!waittoread(sock)){ // no data incoming
            //DBG("no incoming data");
            continue;
        }
        if((rd = read(sock, buff, BUFLEN-1)) < 1){
            //DBG("socket closed. Exit");
            break;
        }
        //DBG("Got %zd bytes", rd);
        // add trailing zero to be on the safe side
        buff[rd] = 0;
        // now we should check what do user want
        char *got, *found = buff;
        if((got = stringscan(buff, "GET")) || (got = stringscan(buff, "POST"))){ // web query
            webquery = 1;
            char *slash = strchr(got, '/');
            if(slash) found = slash + 1;
            // web query have format GET /some.resource
        }
        // here we can process user data
        //DBG("user send: %s%s\n", buff, webquery ? ", web" : "");
        // empty request == focus request
        if(strlen(found) < 1 || getparam(S_CMD_FOCUS)){
            //DBG("position request");
            snprintf(buff, BUFLEN, "%.03f", curPos());
        }else if(getparam(S_CMD_STOP)){
            DBG("Stop request");
            emerg_stop = TRUE;
            pthread_mutex_lock(&canbus_mutex);
            pthread_mutex_lock(&moving_mutex);
            if(stop()) sprintf(buff, S_ANS_ERR);
            else sprintf(buff, S_ANS_OK);
            emerg_stop = FALSE;
            pthread_mutex_unlock(&moving_mutex);
            pthread_mutex_unlock(&canbus_mutex);
        }else if(getparam(S_CMD_TARGSPEED)){
            char *ch = strchr(found, '=');
            double spd;
            if(pthread_mutex_trylock(&moving_mutex)) sprintf(buff,  S_ANS_MOVING);
            else{
                pthread_mutex_lock(&canbus_mutex);
                if(!ch || !str2double(&spd, ch+1) || fabs(spd) < MINSPEED || fabs(spd) > MAXSPEED || movewconstspeed(spd)) sprintf(buff, S_ANS_ERR);
                else{
                    DBG("Move with constant speed %g request", spd);
                    sprintf(buff, S_ANS_OK);
                }
                pthread_mutex_unlock(&canbus_mutex);
                pthread_mutex_unlock(&moving_mutex);
            }
        }else if(getparam(S_CMD_GOTO)){
            char *ch = strchr(found, '=');
            double pos;
            if(!ch || !str2double(&pos, ch+1) || pos < FOCMIN_MM || pos > FOCMAX_MM) sprintf(buff, S_ANS_ERR);
            else{
                DBG("Move to position %g request", pos);
                sprintf(buff, startmoving(pos));
            }
        }else if(getparam(S_CMD_STATUS)){
            const char *msg = S_STATUS_ERROR;
            switch(get_status()){
                case STAT_OK:
                    msg = S_STATUS_OK;
                break;
                case STAT_DAMAGE:
                    msg = S_STATUS_DAMAGE;
                break;
                case STAT_ERROR:
                    msg = S_STATUS_ERROR;
                break;
                case STAT_ESW:
                    msg = S_STATUS_ESW;
                break;
                case STAT_GOFROMESW:
                    msg = S_STATUS_GOFROMESW;
                break;
                case STAT_FORBIDDEN:
                    msg = S_STATUS_FORBIDDEN;
                break;
                default:
                    msg = "Unknown status";
            }
            sprintf(buff, msg);
        }else sprintf(buff, S_ANS_ERR);
        if(!send_data(sock, webquery, buff)){
            WARNX("can't send data, some error occured");
        }
    }
    close(sock);
    //DBG("closed");
    //putlog("socket closed, exit");
    pthread_exit(NULL);
    return NULL;
#undef getparam
}

// main socket server
static void *server(void *asock){
    putlog("server(): getpid: %d, pthread_self: %lu, tid: %lu",getpid(), pthread_self(), syscall(SYS_gettid));
    int sock = *((int*)asock);
    if(listen(sock, BACKLOG) == -1){
        putlog("listen() failed");
        WARN("listen");
        return NULL;
    }
    while(1){
        socklen_t size = sizeof(struct sockaddr_in);
        struct sockaddr_in their_addr;
        int newsock;
        if(!waittoread(sock)) continue;
        newsock = accept(sock, (struct sockaddr*)&their_addr, &size);
        if(newsock <= 0){
            putlog("accept() failed");
            WARN("accept()");
            continue;
        }
        struct sockaddr_in* pV4Addr = (struct sockaddr_in*)&their_addr;
        struct in_addr ipAddr = pV4Addr->sin_addr;
        char str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &ipAddr, str, INET_ADDRSTRLEN);
        //DBG("Got connection from %s", str);
        pthread_t handler_thread;
        if(pthread_create(&handler_thread, NULL, handle_socket, (void*) &newsock)){
            putlog("server(): pthread_create() failed");
            WARN("pthread_create()");
        }else{
            //DBG("Thread created, detouch");
            pthread_detach(handler_thread); // don't care about thread state
        }
    }
    putlog("server(): UNREACHABLE CODE REACHED!");
}

// data gathering & socket management
static void daemon_(int sock){
    if(sock < 0) return;
    pthread_t sock_thread;
    if(pthread_create(&sock_thread, NULL, server, (void*) &sock)){
        putlog("daemon_(): pthread_create() failed");
        ERR("pthread_create()");
    }
    do{
        if(pthread_kill(sock_thread, 0) == ESRCH){ // died
            WARNX("Sockets thread died");
            putlog("Sockets thread died");
            pthread_join(sock_thread, NULL);
            if(pthread_create(&sock_thread, NULL, server, (void*) &sock)){
                putlog("daemon_(): new pthread_create() failed");
                ERR("pthread_create()");
            }
        }
        usleep(500000); // sleep a little or thread's won't be able to lock mutex
        // get current position
        if(!pthread_mutex_trylock(&canbus_mutex)){
            getPos(NULL);
            sysstatus st = get_status();
            pthread_mutex_unlock(&canbus_mutex);
            if(st != STAT_OK){
                getoutESW();
            }
        }
    }while(1);
    putlog("daemon_(): UNREACHABLE CODE REACHED!");
}

/**
 * Run daemon service
 */
void daemonize(const char *port){
    int sock = -1;
    struct addrinfo hints, *res, *p;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    if(getaddrinfo(NULL, port, &hints, &res) != 0){
        ERR("getaddrinfo");
    }
    struct sockaddr_in *ia = (struct sockaddr_in*)res->ai_addr;
    char str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(ia->sin_addr), str, INET_ADDRSTRLEN);
    DBG("canonname: %s, port: %u, addr: %s\n", res->ai_canonname, ntohs(ia->sin_port), str);
    // loop through all the results and bind to the first we can
    for(p = res; p != NULL; p = p->ai_next){
        if((sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1){
            WARN("socket");
            continue;
        }
        int reuseaddr = 1;
        if(setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(int)) == -1){
            ERR("setsockopt");
        }
        if(bind(sock, p->ai_addr, p->ai_addrlen) == -1){
            close(sock);
            WARN("bind");
            continue;
        }
        break; // if we get here, we have a successfull connection
    }
    if(p == NULL){
        putlog("failed to bind socket, exit");
        // looped off the end of the list with no successful bind
        ERRX("failed to bind socket");
    }
    freeaddrinfo(res);
    daemon_(sock);
    close(sock);
    putlog("socket closed, exit");
    signals(0);
}

/**************** CLIENT FUNCTIONS ****************/

/**
 * @brief sock_send_data - send data to a socket
 */
void sock_send_data(const char *host, const char *port, const char *data){
    int sock = 0;
    struct addrinfo hints, *res, *p;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    if(getaddrinfo(host, port, &hints, &res) != 0){
        ERR("getaddrinfo");
    }
    struct sockaddr_in *ia = (struct sockaddr_in*)res->ai_addr;
    char str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(ia->sin_addr), str, INET_ADDRSTRLEN);
    DBG("canonname: %s, port: %u, addr: %s\n", res->ai_canonname, ntohs(ia->sin_port), str);
    // loop through all the results and bind to the first we can
    for(p = res; p != NULL; p = p->ai_next){
        if((sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1){
            WARN("socket");
            continue;
        }
        if(connect(sock, p->ai_addr, p->ai_addrlen) == -1){
            WARN("connect()");
            close(sock);
            continue;
        }
        break; // if we get here, we have a successfull connection
    }
    if(p == NULL) ERRX("failed to connect to server");
    size_t L = strlen(data);
    if(send(sock, data, L, 0) != (ssize_t)L){ WARN("send"); return;}
    double t0 = dtime();
    while(dtime() - t0 < SOCKET_TIMEOUT){
        if(!waittoread(sock)) continue;
        char buff[32];
        int n = read(sock, buff, 31);
        if(n > 0){
            buff[n] = 0;
            printf("%s\n", buff);
            close(sock);
            return;
        }
    }
    WARN("No answer!");
}
