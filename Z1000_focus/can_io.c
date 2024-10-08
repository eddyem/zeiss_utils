/* CAN I/O library (for compatibility with the old one,      */
/*                  but through the new SocketCAN interface) */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

#include "can_io.h"

char can_dev[40] = "/dev/can0";/* for compatibility (only "can0" needs) */
static int can_sck = -1;       /* can raw socket */
static struct timeval start_tv, tv;
static double start_time;

void set_sending_mode(int x) {return;}
int can_sending_mode() {return(0);}

void *init_can_io() {
    struct sockaddr_can addr;
    struct canfd_frame frame;
    struct ifreq ifr;

    /* open socket */
    if ((can_sck = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
	perror("CAN socket");
	can_exit(0);
    }

    strncpy(ifr.ifr_name, &can_dev[5], IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex) {
	perror("if_nametoindex");
	can_exit(0);
    }
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(can_sck, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
	perror("bind CAN socket");
	can_exit(0);
    }

    gettimeofday(&start_tv, NULL);
    start_time = (double)start_tv.tv_sec + (double)start_tv.tv_usec/1e6;
    tv.tv_sec = tv.tv_usec = 0;


    signal(SIGHUP, can_exit);
    signal(SIGINT, can_exit);
    signal(SIGQUIT,can_exit);
    signal(SIGTERM,can_exit);

    return(&can_dev[5]);
}

int can_ok() {
   return(can_sck>0);
}

/* wait for CAN-frame  */
int can_wait(int fd, double tout)
{
    int nfd,width;
    fd_set readfds;

    if(fd==0 && tout>=0.01) {
	double dt = can_dsleep(tout);
	if(dt>0.) can_dsleep(dt);
	return(0);
    }
    if(fd<0) fd=can_sck;
    if(fd>0) {
	FD_ZERO(&readfds);
	FD_SET(fd, &readfds);
	width = fd+1;
    } else
	width = 0;
    tv.tv_sec = (int)tout;
    tv.tv_usec = (int)((tout - tv.tv_sec)*1000000.+0.9);

    if(fd>0 && can_sck>0)
	nfd = select(width, &readfds, (fd_set *)NULL, (fd_set *)NULL,  &tv);
    else
	nfd = select(0, (fd_set *)NULL, (fd_set *)NULL, (fd_set *)NULL,  &tv);
    if(nfd < 0) {
	if(errno != EINTR)
	    perror("Error in can_wait(){ select() }");
	return(-1);
    } else if(nfd == 0)                 /* timeout! */
	return(0);
    if(fd>0 && FD_ISSET(fd, &readfds))  /* Rx frame! */
	return(1);
    return(0);
}

/* for compatibility with my old can-library  */
void can_clean_recv(int *psock, double *rtime) {
    struct timeval tmv;
    struct timezone tz;
    gettimeofday(&tmv,&tz);
    *rtime = tmv.tv_sec + (double)tmv.tv_usec/1000000.;
    *psock = can_sck;
    if(can_sck>0) {
	int n=0;
	struct can_frame frame;
	fcntl(can_sck, F_SETFL, O_NONBLOCK);
	do {
	    n=recv(can_sck, &frame, sizeof(struct can_frame),0);
	} while(n>0);
	if(n<0 && errno != EAGAIN) {
	    perror("recv from CAN-socket"); fflush(stderr);
	}
    }
}

int can_recv_frame(int *psock, double *rtime,
		  canid_t *id, int *length, unsigned char data[]) {
    int i,n=0;
    struct can_frame frame;
    struct pollfd pfd;
    if(*psock > 0) {
	pfd.fd = *psock;
	pfd.events=POLLIN;
	pfd.revents=0;
	if((n=poll(&pfd,1,1))<0) {
	    perror("CAN-socket poll() error"); fflush(stderr);
	    return(0);
	}
	if(n==0) return(0);
	n=recv(*psock, &frame, sizeof(struct can_frame),0);
	if(n<0 && errno != EAGAIN) {
	    perror("recv frame from CAN-socket"); fflush(stderr);
	} else if(n>0) {
	    if(frame.len>8) frame.len=8; // no CAN FD frames in our systems!
	    *id = frame.can_id;
	    *length = frame.len;
	    for(i = 0; i < frame.len; i++)
		data[i] = frame.data[i];
	    if(ioctl(*psock, SIOCGSTAMP, &tv)<0) {
		perror("ioctl(to get frame timestamp)"); fflush(stderr);
	    } else
		*rtime = tv.tv_sec + (double)tv.tv_usec/1000000.;
	    return(1);
	}
    }
    return(0);
}
/* send tx-frame from client process */
int can_send_frame(canid_t id, int length, unsigned char data[]) {
    int i, ret=1;
    struct can_frame frame;
    if(can_sck<0)
       return(-1);
    if(length>8) length=8;
    if(length<0) length=0;
    memset(&frame, 0, sizeof(struct can_frame)); /* init CAN frame, e.g. LEN = 0 */
    frame.can_id = id;
    frame.len = length;
    for(i=0;i<length;i++) frame.data[i]=data[i];
    if(send(can_sck, &frame, sizeof(struct can_frame),0)<0) {
	perror("send frame to CAN-socket"); fflush(stderr);
    }
    return(ret);
}

void can_exit(int sig) {
    int ret;
    char ss[12];

    if(sig) signal(sig,SIG_IGN);
    switch (sig) {
	case 0      : strcpy(ss,"Exiting -");  break;
	case SIGHUP : strcpy(ss,"SIGHUP -");  break;
	case SIGINT : strcpy(ss,"SIGINT -");  break;
	case SIGQUIT: strcpy(ss,"SIGQUIT -"); break;
	case SIGFPE : strcpy(ss,"SIGFPE -");  break;
	case SIGPIPE: strcpy(ss,"SIGPIPE -"); break;
	case SIGSEGV: strcpy(ss,"SIGSEGV -"); break;
	case SIGTERM: strcpy(ss,"SIGTERM -"); break;
	default:  sprintf(ss,"SIG_%d -",sig); break;
    }
    switch (sig) {
	default:
	case SIGHUP :
	     can_prtime(stderr);
	     fprintf(stderr,"%s Ignore .....\n",ss);
	     fflush(stderr);
	     signal(sig, can_exit);
	     return;
	case 0:
	case SIGINT :
	case SIGPIPE:
	case SIGQUIT:
	case SIGFPE :
	case SIGSEGV:
	case SIGTERM:
	     if(can_sck>=0) close(can_sck);
	     can_prtime(stderr);
	     fprintf(stderr,"%s process stop!\n",ss);
	     fflush(stderr);
	     exit(sig);
    }
}

char *time2asc(double t)
{
 static char stmp[10][20];
 static int itmp=0;
    char *lin = stmp[itmp];
    int h, min;
    double sec;
    h   = (int)(t/3600.);
    min = (int)((t - (double)h*3600.)/60.);
    sec = t - (double)h*3600. - (double)min*60.;
    h %= 24;
    sprintf(lin, "%02d:%02d:%09.6f", h,min,sec);
    itmp = (itmp+1)%10;
    return lin;
}

double can_dsleep(double dt) {
   struct timespec ts,tsr;
   ts.tv_sec = (time_t)dt;
   ts.tv_nsec = (long)((dt-ts.tv_sec)*1e9);
   nanosleep(&ts,&tsr);
   return((double)ts.tv_sec + (double)ts.tv_nsec/1e9);
}

double can_dtime() {
   struct timeval ct;
   struct timezone tz;
   gettimeofday(&ct, &tz);
   return ((double)ct.tv_sec + (double)ct.tv_usec/1e6);
}

char *can_atime() {return(time2asc(can_dtime()));}

void can_prtime(FILE *fd) {
   static double otime=0.0;
   double ntime=can_dtime();
   time_t itime = (int)ntime;
   if(otime==0.0) tzset();
   ntime -= (double)timezone;
   if((((int)ntime)%(24*3600) < ((int)otime)%(24*3600)) || otime==0.0)
      fprintf(fd,"========================\n%s",ctime(&itime));
   fprintf(fd,"%s ",time2asc(ntime));
   otime=ntime;
}

