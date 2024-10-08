/*
 * This file is part of the Zphocus project.
 * Copyright 2019 Edward V. Emelianov <edward.emelianoff@gmail.com>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <math.h>  // fabs
#include <sys/wait.h>
#include <sys/prctl.h>
#include <signal.h>
#include "can_io.h"
#include "can_encoder.h"
#include "canopen.h"
#include "checkfile.h"
#include "cmdlnopts.h"
#include "HW_dependent.h"
#include "socket.h"
#include "usefull_macros.h"

glob_pars *G;

/**
 * @brief verbose - printf when parameter `verbose` set
 * @param fmt - format & other attrs
 * @return amount of printed characters
 */
int verbose(const char *fmt, ...){
    if(!G || !G->verbose) return 0;
    va_list ar; int i;
    va_start(ar, fmt);
    i = vprintf(fmt, ar);
    va_end(ar);
    return i;
}

void signals(int signo){
    unlink_pidfile();
    can_exit(signo);
}

static void cmdparser(){
#define BUFL 128
    char buf[BUFL] = {0,};
    if(G->stop) sprintf(buf, S_CMD_STOP);
    else if(fabs(G->targspeed) > DBL_EPSILON) snprintf(buf, BUFL, S_CMD_TARGSPEED "=%g", G->targspeed);
    else if(!isnan(G->gotopos)) snprintf(buf, BUFL, S_CMD_GOTO "=%g", G->gotopos);
    else sprintf(buf, S_CMD_FOCUS);
    sock_send_data(G->host, G->port, buf);
#undef BUFL
}

extern char can_dev[40];

int main (int argc, char *argv[]){
    int ret = 0;
    double curposition = 0;
    initial_setup();
    G = parse_args(argc, argv);

    if(fabs(G->targspeed) > DBL_EPSILON && !isnan(G->gotopos))
        ERRX("Arguments \"target speed\" and \"target position\" can't meet together!");
    if(fabs(G->targspeed) > DBL_EPSILON){
        if(G->nomotor) ERRX("Can't set target speed without motor");
        if(fabs(G->targspeed) < MINSPEED || fabs(G->targspeed) > MAXSPEED){
            WARNX("Target speed should be be from %d to %d (rev/min)", MINSPEED, MAXSPEED);
            return 1;
        }
    }
    if(!isnan(G->gotopos)){
        if(G->noencoder || G->nomotor) ERRX("Can't go to target position without encoder or motor");
        if(G->gotopos > FOCMAX_MM || G->gotopos < FOCMIN_MM){
            WARNX("Focal distance may be from %g to %g mm", FOCMIN_MM, FOCMAX_MM);
            return 1;
        }
    }

    signal(SIGTSTP, SIG_IGN);
    signal(SIGHUP, SIG_IGN);
    can_dev[8] = '1';

    if(G->server){ // daemonize & run server
#ifndef EBUG
        if(daemon(1, 0)){
            ERR("daemon()");
        }
#endif
        check4running(G->pidfilename);
#ifndef EBUG
        while(1){ // guard for dead processes
            pid_t childpid = fork();
            if(childpid){
                DBG("child: %d", childpid);
                wait(NULL);
                DBG("child: %d DIED", childpid);
                sleep(1);
            }else{
                prctl(PR_SET_PDEATHSIG, SIGTERM); // send SIGTERM to child when parent dies
                if(G->server || G->standalone){ // init hardware
                    if(!G->noencoder && init_encoder(G->nodenum, G->reset)) ERRX("Encoder not found");
                    if(!G->nomotor && init_motor_ids(G->motorID)) ERRX("Error during motor initialization");
                }
                if(G->logname){ // open log file in child
                    openlogfile(G->logname);
                    putlog("created child with PID %d", getpid());
                }
                daemonize(G->port);
            }
        }
#else
    if(G->logname){ // open log file in child
        openlogfile(G->logname);
        putlog("Running in debug mode, PID %d", getpid());
    }
    daemonize(G->port);
#endif
    }else if(!G->standalone){
        cmdparser();
        return 0;
    }

    check4running(G->pidfilename);

    if(!G->noencoder && init_encoder(G->nodenum, G->reset)){
        WARNX("Encoder not found");
#ifndef EBUG
        return 1;
#endif
    }
    if(!G->nomotor && init_motor_ids(G->motorID)){
        WARNX("Error during motor initialization");
        ret = 1;
#ifndef EBUG
        goto Oldcond;
#endif
    }
    if(getPos(&curposition)){
        WARNX("Can't read current position");
        ret = 1;
        goto Oldcond;
    }else verbose("Position @ start: %.2fmm\n", curposition);

    if(fabs(G->monitspd) > DBL_EPSILON){
        movewithmon(G->monitspd);
        goto Oldcond;
    }

    if(G->stop){ // Stop motor
        if(stop()) ret = 1;
        goto Oldcond;
    }

    if(fabs(G->targspeed) > DBL_EPSILON){ // move with constant speed
        verbose("Try to move with %g revolutions per minute\n", G->targspeed);
        if(movewconstspeed(G->targspeed)){
            ret = 1;
            goto Oldcond;
        }
    }

    if(!isnan(G->gotopos)){ // move to given position
        verbose("Try to move to position %g\n", G->gotopos);
        ret = move2pos(G->gotopos);
        goto Oldcond;
    }
    double spd;
    unsigned long pos;

Oldcond:
    if(get_pos_speed(&pos, &spd)) WARNX("Can't read current position");
    else{
        curposition = FOC_RAW2MM(pos);
        verbose("speed=%d\n", spd);
        if(G->verbose) printf("pos=%.03fmm, ", curposition);
        else printf("%.03f\n", curposition);
    }
    if(G->showesw){
        eswstate e;
        if(CAN_NOERR == get_endswitches(&e)) switch(e){
            case ESW_INACTIVE:
                green("End-switches inactive\n");
            break;
            case ESW_CW_ACTIVE:
                red("Active CW end-switch\n");
            break;
            case ESW_CCW_ACTIVE:
                red("Active CCW end-switch\n");
            break;
            case ESW_BOTH_ACTIVE:
            default:
                red("ERROR: both end-switches active\n");
        }
    }
    returnPreOper(G->chpresetval);
    return ret;
}
