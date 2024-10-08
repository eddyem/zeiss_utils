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

#include "DS406_canopen.h"
#include "HW_dependent.h"
#include "can_encoder.h"
#include "canopen.h"
#include "motor_cancodes.h"
#include "socket.h"
#include "usefull_macros.h"
#include <math.h>   // fabs
#include <string.h> // memcpy

// flags: encoder is ready, motor is ready
static uint8_t encoderRDY = 0, motorRDY = 0;

// printf when -v
extern int verbose(const char *fmt, ...);

extern bool emerg_stop;

// CAN bus IDs: for motor's functions (PI ID [F=4] == PO ID[F=3] + 1) and parameters
static unsigned long motor_id = 0, motor_p_id = 0;//, bcast_id = 1;
// current motor position (RAW)
static unsigned long curposition = 0;
// encoder's node number
static int encnodenum = 0;
// system status
static sysstatus curstatus = STAT_OK;
// current raw motor speed (without MOTOR_REVERSE)
static int16_t targspd = 0;

static canstatus can_write_par(uint8_t subidx, uint16_t idx, uint32_t *parval);
static canstatus can_read_par(uint8_t subidx, uint16_t idx, uint32_t *parval);
static int move(unsigned long targposition, int16_t rawspeed);
static int waitTillStop();

// check if end-switches are in default state
// return 0 if all OK
static int chk_eswstates(){
    if(!motorRDY) return 0;
    FNAME();
    uint32_t cw, ccw;
    if(CAN_NOERR != can_read_par(PAR_DI_SUBIDX, PAR_CW_IDX, &cw)) goto verybad;
    if(CAN_NOERR != can_read_par(PAR_DI_SUBIDX, PAR_CCW_IDX, &ccw)) goto verybad;
    uint32_t parval = DI_ENSTOP;
    if(cw != DI_ENSTOP || ccw != DI_ENSTOP){ // activate enable/stop
        WARNX("The end-switches state wasn't default!");
        if(waitTillStop()) return 1; // we can change motor parameters only in stopped state
        if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, PAR_CW_IDX, &parval)) goto verybad;
        if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, PAR_CCW_IDX, &parval)) goto verybad;
    }
    return 0;
verybad:
    curstatus = STAT_ERROR;
    return 1;
}

/**
 * @brief chkMove - check whether moving available
 * @param spd - speed (used only its sign) before MOTOR_REVERSE!
 * @return 0 if all OK
 */
static int chkMove(int spd){
    //FNAME();
    if(!motorRDY){
        curstatus = STAT_ESW;
        return 1;
    }
    if(curstatus == STAT_DAMAGE){
        SINGLEWARN(WARN_MOVEDAMAGED);
        return 1;
    }else clrwarnsingle(WARN_MOVEDAMAGED);
    eswstate e;
    if(CAN_NOERR != get_endswitches(&e)){
        curstatus = STAT_ERROR;
        return 1;
    }
    if(e == ESW_INACTIVE){
        double posmm;
        if(!encoderRDY) return 0;
        if(getPos(&posmm)) return 1;
        if(posmm <= FOCMIN_MM && spd < 0){
            WARNX("Try to move to the left of minimal position");
            return 1;
        }
        if(posmm >= FOCMAX_MM && spd > 0){
            WARNX("Try to move to the right of maximal position");
            return 1;
        }
        return 0;
    }
    if(e == ESW_BOTH_ACTIVE){
        SINGLEWARN(WARN_BOTHESW);
        curstatus = STAT_DAMAGE;
        return 1;
    }else clrwarnsingle(WARN_BOTHESW);
    if(e == ESW_CCW_ACTIVE && spd < 0){
        curstatus = STAT_ESW;
        WARNX("Try to move over the CCW end-switch");
        return 1;
    }else if(e == ESW_CW_ACTIVE && spd > 0){
        curstatus = STAT_ESW;
        WARNX("Try to move over the CW end-switch");
        return 1;
    }
    return 0;
}

/**
 * @brief init_encoder - encoder's interface initialisation
 * @param encnode - encoder's node number
 * @param reset   - reset node before operating
 * @return 0 if all OK
 */
int init_encoder(int encnode, int reset){
    FNAME();
    unsigned long lval;
    encnodenum = encnode;
    verbose("cur node: %d\n", encnodenum);
    if(!initNode(encnodenum)){
        WARNX("Can't find device with Node address %d!", encnodenum);
        //signals(9);
        return 1;
    }
    if(reset){
        verbose("Reset node %d... ", encnodenum);
        if(resetNode(encnodenum))
            verbose("Ok!\n");
        else
            verbose("Failed.\n");
    }
    if(getLong(encnodenum, DS406_DEVTYPE, 0, &lval)){
        int prof = lval&0xffff;
        int type = lval>>16;
        verbose("Found absolute %s-turn rotary encoder DS%d\n", (type==2)?"multi":"single", prof);
        if(prof != 406 || type != 2){
            WARNX("The device on node %d isn't a multi-turn encoder!", encnodenum);
            return 1;
        }
    }else{
        WARNX("Can't get encoder device type");
        return 1;
    }
    verbose("Set operational... ");
    startNode(encnodenum);
    int n, i = recvNextPDO(0.1, &n, &lval);
    int state = getNodeState(encnodenum);
    verbose("State=%02x\n", state);
    if(state == NodeOperational) verbose("Ok!\n");
    else{
        WARNX("Failed to start node!");
        returnPreOper(-1);
        return 1;
    }
    if(i > 0) verbose("Node%d PDO%d %08lx\n",n,i,lval);
    do{
        int node[4], pdo_n[4];
        unsigned long pdo_v[4];
        verbose("Send SYNC...\n");
        clean_recv();
        sendSync();
        can_dsleep(0.01);
        if((n = recvPDOs(0.5, 4, node, pdo_n, pdo_v))==0)
            WARNX("No PDO received on SYNC");
        else for(i=0;i<n;i++)
            verbose("Node%d PDO%d %08lx (%ld)\n",node[i],pdo_n[i],pdo_v[i],pdo_v[n]);
    }while(0);
    curstatus = STAT_OK;
    encoderRDY = 1;
    return 0;
}

/**
 * @brief go_out_from_ESW - check ESW roles, check ESW states and try to go out if stay on ESW
 * @return 0 if all OK
 */
int go_out_from_ESW(){
    //FNAME();
    if(!encoderRDY || !motorRDY) return 0;
    uint32_t parval;
    if(chk_eswstates()) return 1;
    // check esw
    eswstate e;
    getPos(NULL);
    if(CAN_NOERR != get_endswitches(&e)){
        return 1;
    }
    if(e == ESW_BOTH_ACTIVE){ // error situation!
        SINGLEWARN(WARN_BOTHESW);
        if(curstatus != STAT_DAMAGE){
            curstatus = STAT_DAMAGE;
        }
        return 1;
    }else clrwarnsingle(WARN_BOTHESW);
    if(e == ESW_INACTIVE){
        DBG("Esw inactive");
        int r = 0;
// TODO: fix trouble with current position if it is over available position
  //      if(curposition < FOCMIN) r = move(FOCMIN, MAXSPEED);
  //      else if(curposition > FOCMAX) r = move(FOCMAX, -MAXSPEED);
        curstatus = STAT_OK;
        return r;
    }
    // check that current position is in available zone
    if(e == ESW_CW_ACTIVE && curposition < FOCPOS_CW_ESW){
        // CW end-switch activated in forbidden zone
        if(curstatus != STAT_DAMAGE){
            WARNX("CW end-switch in forbidden zone (to the left of normal position)!");
            curstatus = STAT_DAMAGE;
        }
        return 1;
    }else if(e == ESW_CCW_ACTIVE && curposition > FOCPOS_CCW_ESW){
        // CCW end-switch activated in forbidden zone
        if(curstatus != STAT_DAMAGE){
            WARNX("CCW end-switch in forbidden zone (too far)!");
            curstatus = STAT_DAMAGE;
        }
        return 1;
    }
    curstatus = STAT_GOFROMESW;
    // try to move from esw
    parval = DI_NOFUNC;
    WARNX("Try to move from ESW");
    uint16_t idx = (e == ESW_CW_ACTIVE) ? PAR_CW_IDX : PAR_CCW_IDX;
    if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, idx, &parval)) goto bad; // turn off motor stopping @esw
    int16_t speed = (e == ESW_CW_ACTIVE) ? -RAWSPEED(ESWSPEED) : RAWSPEED(ESWSPEED);
    for(int i = 0; i < 5; ++i){ // 5 tries to move out
        getPos(NULL);
        DBG("try %d, pos: %lu, E=%d", i, curposition, e);
        unsigned long targ = (e == ESW_CW_ACTIVE) ? curposition - (double)FOCSCALE_MM*0.2 : curposition + (double)FOCSCALE_MM*0.2;
        if(targ > FOCMAX) targ = FOCMAX;
        else if(targ < FOCMIN) targ = FOCMIN;
        if(move(targ, speed)) continue;
        get_endswitches(&e);
        if(e == ESW_INACTIVE) break;
    }
    if(chk_eswstates()) return 1;
    curstatus = STAT_OK;
    return 0;
bad:
    WARNX("Can't move out from end-switch");
    curstatus = STAT_ERROR;
    return 1;
}

/**
 * @brief init_motor_ids - convert motor address into CAN IDs (PO & data)
 *          run this function before any working with motor
 * @param addr - motor driver address
 * @return 0 if all OK
 */
int init_motor_ids(int addr){
    if(addr < 0 || addr > 0x3f){
        WARNX("Wrong motor address, should be from 0 to 63");
        return 1;
    }
    motor_id = MOTOR_PO_ID(addr);
    motor_p_id = MOTOR_PAR_ID(addr);
    DBG("motor POid=%lu, motor_PROCid=%lu", motor_id, motor_p_id);
    motorRDY = 1;
    // check esw roles & end-switches state
    if(go_out_from_ESW()) return 1;
    return 0;
}

/**
 * @brief getPos - get current encoder's position
 * @param pos (o) - position value (in mm)
 * @return 0 if all OK
 */
int getPos(double *pos){
    //FNAME();
    if(!encoderRDY) return 1;
    int r = !(getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition));
    double posmm = FOC_RAW2MM(curposition);
    if(pos) *pos = posmm;
    verbose("Raw position: %ld\nposition in mm: %.2f\n", curposition, posmm);
    eswstate e;
    if(CAN_NOERR != get_endswitches(&e)){
        curstatus = STAT_ERROR;
    }else switch(e){
        case ESW_BOTH_ACTIVE:
            WARNX("Damage state: both end-switches are active");
            curstatus = STAT_DAMAGE;
        break;
        case ESW_CCW_ACTIVE:
            if(posmm > FOCMIN_MM + ESW_DIST_ALLOW){
                WARNX("Damage state: CCW end-switch in forbidden zone");
                curstatus = STAT_DAMAGE;
            }else curstatus = STAT_ESW;
        break;
        case ESW_CW_ACTIVE:
            if(posmm < FOCMAX_MM - ESW_DIST_ALLOW){
                WARNX("Damage state: CW end-switch in forbidden zone");
                curstatus = STAT_DAMAGE;
            }else curstatus = STAT_ESW;
        break;
        case ESW_INACTIVE:
        default:
            curstatus = STAT_OK;
    }
    //DBG("targspd = %d", targspd);
    if(targspd){
        if(posmm <= FOCMIN_MM && targspd < 0){ // bad value
            SINGLEWARN(WARN_LESSMIN);
            stop();
            curstatus = STAT_FORBIDDEN;
        }else if(posmm >= FOCMAX_MM && targspd > 0){
            SINGLEWARN(WARN_GRTRMAX);
            stop();
            curstatus = STAT_FORBIDDEN;
        }else{
            clrwarnsingle(WARN_LESSMIN);
            clrwarnsingle(WARN_GRTRMAX);
        }
    }else{
        if(posmm <= FOCMIN_MM){
            stop();
            //WARNX("Current position <= FOCMIN");
        }else if(posmm >= FOCMAX_MM){
            stop();
            //WARNX("Current position >= FOCMAX");
        }
    }
    return r;
}

// just return value of curposition in mm
double curPos(){
    return FOC_RAW2MM(curposition);
}

/**
 * @brief returnPreOper - return encoder into pre-operational state
 * @arg presetval - new preset value (if > -1)
 */
void returnPreOper(long long presetval){
    if(!encoderRDY) return;
    verbose("Return to Pre-operational... ");
    setPreOper(encnodenum);
    int state=getNodeState(encnodenum);
    verbose("State=0x%02x - ", state);
    if(state != NodePreOperational){
        verbose("Failed!\n");
        return;
    }
    verbose("Ok!\n");
    encoderRDY = 0;
    if(presetval < 0) return;
    green("Try to change preset value to %lld", presetval);
    printf("\n");
    if(presetval > UINT32_MAX){
        WARNX("Value %lld too big", presetval);
    }
    unsigned long val = (uint32_t) presetval, nval = ~val;
    // set "configuration valid"
    /*
    if(!setByte(encnodenum, DS406_CONF_VALID, 0, DS406_CONF_VALID_VALID)){
        WARNX("Can't switch to setup mode");
        return;
    }
    */
    // Safety code sequence
    if(!setLong(encnodenum, DS406_CONF_PARAMETERS, 2, val)){
        WARNX("Can't change safety code sequence value");
        return;
    }
    if(!setByte(encnodenum, DS406_CONF_VALID, 0, DS406_CONF_VALID_VALID)){
        WARNX("Can't switch to setup mode");
        return;
    }
    if(!setLong(encnodenum, DS406_CONF_PARAMETERS, 3, nval)){
        WARNX("Can't change inverted safety code sequence value");
        return;
    }
}

/**
 * @brief fix_targspeed - fix speed value if it is greater MAXSPEED or less than MINSPEED
 * @param targspd (io) - target speed in rev/min
 */
static void fix_targspeed(int16_t *targspd){
    if(!targspd) return;
    DBG("cur spd: %d", *targspd);
    int16_t sign = (*targspd < 0) ? -1 : 1;
    int16_t absspd = abs(*targspd);
    if(absspd < MINSPEED) *targspd = sign * MINSPEED;
    else if(absspd > MAXSPEED) *targspd = sign * MAXSPEED;
    DBG("become spd: %d", *targspd);
}

/**
 * @brief can_send_chk - send CAN frame to motor & check answer
 * @param buf (i)  - PO data frame (6 bytes)
 * @param obuf (o) - received PI data frame
 * @return status
 */
static canstatus can_send_chk(unsigned char *buf, unsigned char *obuf){
    if(!motorRDY) return CAN_NOANSWER;
    const int l = 6; // frame length
    /*if(G->verbose){
        printf("Send frame to ID=%lu: ", motor_id);
        for(int i = 0; i < l; ++i)
            printf(" %02x", buf[i]);
        printf("\n");
    }*/
    if(can_send_frame(motor_id, l, buf) <= 0){
        SINGLEWARN(WARN_CANSEND);
        return CAN_CANTSEND;
    }else clrwarnsingle(WARN_CANSEND);
    int I, rxpnt, dlen;
    canid_t idr;
    double rxtime;
    unsigned char rdata[8];
    can_clean_recv(&rxpnt, &rxtime);
    for(I = 0; I < 50; ++I){
        if(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata) && (idr&0x1fffffff) == motor_id+1) break;
        can_dsleep(0.01);
    }
    if(I == 50){
        SINGLEWARN(WARN_CANNOANS);
        return CAN_NOANSWER;
    }else clrwarnsingle(WARN_CANNOANS);
    if(obuf) memcpy(obuf, rdata, l);
    /*if(G->verbose){
        printf("Got answer with ID=%d: ", idr&0x1fffffff);
        for(int i = 0; i < dlen; ++i)
            printf(" %02x", rdata[i]);
        printf("\n");
    }*/
    if((rdata[0] & (SW_B_MAILFUN|SW_B_READY)) == SW_B_MAILFUN){ // error
        WARNX("Mailfunction, error code: %d", rdata[1]);
        return CAN_ERROR; // error
    }else if((rdata[0] & (SW_B_MAILFUN|SW_B_READY)) == (SW_B_MAILFUN|SW_B_READY)){ // warning
        WARNX("Warning, code: %d", rdata[1]);
        buf[1] |= CW_B_CLERR; // try to clear warnings
        can_send_frame(motor_id, l, buf);
        return CAN_WARNING; // warning
    }
    //verbose("Got motor state: %d\n", rdata[0]);
    return 0;
}

/**
 * @brief can_send_parag - send/get motor parameters
 * @param buf (i)  - parameter out data frame (8 bytes)
 * @param obuf (o) - parameter in data frame (8 bytes)
 * @return status
 */
static canstatus can_send_param(unsigned char *buf, unsigned char *obuf){
    if(!motorRDY) return CAN_NOANSWER;
    const int l = 8; // frame length
    int I, rxpnt, dlen;
    canid_t idr;
    double rxtime;
    unsigned char rdata[8];
/*
green("Sent param:     ");
for(int i=0; i<l; ++i) printf("0x%02x ", buf[i]);
printf("\n");
*/
    if(can_send_frame(motor_p_id, l, buf) <= 0){
        SINGLEWARN(WARN_CANSEND);
        return CAN_CANTSEND;
    } else clrwarnsingle(WARN_CANSEND);
    can_clean_recv(&rxpnt, &rxtime);
    for(I = 0; I < 50; ++I){
        if(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata) && (idr&0x1fffffff) == motor_p_id+1) break;
        //DBG("Got frame from ID 0x%x, need: 0x%x", idr&0x1fffffff, motor_p_id+1);
        can_dsleep(0.01);
    }
    if(I == 50){
        SINGLEWARN(WARN_SENDPAR);
        return CAN_NOANSWER;
    }else clrwarnsingle(WARN_SENDPAR);
/*
green("Received param: ");
for(int i=0; i<dlen; ++i) printf("0x%02x ", rdata[i]);
printf("\n");
*/
    if(obuf) memcpy(obuf, rdata, dlen);
    if(obuf[0] & CAN_PAR_ERRFLAG){
        WARNX("Wrong parameter idx/subidx or other error");
        return CAN_WARNING;
    }
    if(*((uint32_t*)buf) != *((uint32_t*)obuf)){
        WARNX("Got wrong answer for parameter request");
        return CAN_WARNING;
    }
    return CAN_NOERR;
}

/**
 * @brief can_read_par - read motor parameter
 * @param subidx     - parameter subindex
 * @param idx        - parameter index
 * @param parval (o) - parameter value
 * @return status
 */
static canstatus can_read_par(uint8_t subidx, uint16_t idx, uint32_t *parval){
    if(!motorRDY) return CAN_NOANSWER;
    if(!parval) return CAN_WARNING;
    uint8_t buf[8] = {CAN_READPAR_CMD, subidx}, obuf[8];
    buf[2] = idx >> 8;
    buf[3] = idx & 0xff;
    canstatus s = can_send_param(buf, obuf);
    if(s != CAN_NOERR) return s;
    *parval = obuf[4]<<24 | obuf[5]<<16 | obuf[6]<<8 | obuf[7];
    return CAN_NOERR;
}

/**
 * @brief can_write_par - write motor parameter
 * @param subidx     - parameter subindex
 * @param idx        - parameter index
 * @param parval (i) - new parameter value (NULL for 0)
 * @return status
 */
static canstatus can_write_par(uint8_t subidx, uint16_t idx, uint32_t *parval){
    if(!motorRDY) return CAN_NOANSWER;
    uint8_t buf[8] = {CAN_WRITEPAR_CMD, subidx,0}, obuf[8];
    buf[2] = idx >> 8;
    buf[3] = idx & 0xff;
    if(parval){
        uint32_t par = *parval;
        DBG("parameter: %d", par);
        buf[4] = (par >> 24) & 0xff;
        buf[5] = (par >> 16) & 0xff;
        buf[6] = (par >> 8)  & 0xff;
        buf[7] = par & 0xff;
    }
    return can_send_param(buf, obuf);
}

/**
 * @brief get_motor_speed
 * @param motstatus (o) - status (if !NULL)
 * @return speed in rev/min
 */
canstatus get_motor_speed(double *spd){
    if(!motorRDY) return CAN_NOANSWER;
    if(!spd) return CAN_WARNING;
    union{
        uint32_t u;
        int32_t i;
    } speed;
    canstatus s = can_read_par(PAR_SPD_SUBIDX, PAR_SPD_IDX, &speed.u);
    if(s != CAN_NOERR){
        return s;
    }
    *spd = (double)speed.i / 1000.;
    return CAN_NOERR;
}

/**
 * @brief get_endswitches - get state of end-switches
 * @param Esw (o) - end-switches state
 * @return
 */
canstatus get_endswitches(eswstate *Esw){
    if(!motorRDY) return CAN_NOANSWER;
    //FNAME();
    uint32_t val = 0;
    canstatus s = can_read_par(PAR_DI_SUBIDX, PAR_DIST_IDX, &val);
    if(s != CAN_NOERR){
        SINGLEWARN(WARN_ESWSTATE);
        return s;
    }else clrwarnsingle(WARN_ESWSTATE);
    if(Esw){
        int v = 0;
        if(!(val & ESW_CW)){ // + pressed
            v |= ESW_CW_ACTIVE;
        }
        if(!(val & ESW_CCW)){ // - pressed
            v |= ESW_CCW_ACTIVE;
        }
        *Esw = v;
    }
    return s;
}

/**
 * @brief stop - stop motor
 * @return 0 if all OK
 */
int stop(){
    if(!motorRDY) return 0;
    //FNAME();
    unsigned char buf[6] = {0, CW_STOP,0,};
    if(can_send_chk(buf, NULL)){
        WARNX("Can't stop motor!");
        return 1;
    }
    targspd = 0;
    return 0;
}

/**
 * @brief waitTillStop - wait for full stop
 * @return 0 if all OK
 */
static int waitTillStop(){
    if(!motorRDY) return 0;
    long oldposition = -1;
    double spd;
    int r = 0;
    if(CAN_NOERR == get_motor_speed(&spd)){
        DBG("speed: %g, targspd: %d", spd, targspd);
        if(fabs(spd) > DBL_EPSILON || targspd) r = stop();
    }else r = stop();
    if(r) r = stop();
    if(!r){
        do{ // wait till stop
            can_dsleep(0.1);
            if(CAN_NOERR == get_motor_speed(&spd)){
                if(fabs(spd) > DBL_EPSILON){
                    //DBG("Still moving, spd=%g", spd);
                    //stop();
                    continue; // wait for zero-speed
                }else{
                    DBG("OK, stopped, spd=%g", spd);
                }
            }else{DBG("can't get motor speed");};
            oldposition = curposition;
            // now wait for full moving stop
            if(!encoderRDY) break;
            getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition);
            //DBG("curpos: %lu, oldpos: %ld", curposition, oldposition);
        }while((long)curposition != oldposition);
    }else{
        curstatus = STAT_ERROR;
        return 1;
    }
    return 0;
}

/**
 * @brief movewconstspeed - move with constant speed
 * @param spd - given speed (rev/min)
 * @return 0 if all OK
 */
int movewconstspeed(int16_t spd){
    if(!motorRDY) return 0;
    if(chkMove(spd)) return 1;
    fix_targspeed(&spd);
    targspd = RAWSPEED(spd);
    unsigned char buf[8] = {0,};
    buf[1] = CW_ENABLE;
    int16_t s = targspd;
    if(MOTOR_REVERSE) s = -s;
    buf[2] = (s >> 8) & 0xff;
    buf[3] = s & 0xff;
    DBG("\tBUF: %d, %d, %d, %d", buf[0], buf[1], buf[2], buf[3]);
    if(can_send_chk(buf, NULL)){
        WARNX("Can't move motor!");
        return 1;
    }
    return 0;
}

/**
 * @brief move - move focuser from current position to approximately `targposition` with speed `rawspeed`
 * @param targposition - target position in raw value
 * @param rawspeed - raw speed value
 * @return 0 if all OK
 */
static int move(unsigned long targposition, int16_t rawspeed){
    if(!motorRDY || !encoderRDY) return 1;
    //FNAME();
    long olddiffr = labs((long)targposition - (long)curposition);
    if(olddiffr < RAWPOS_TOLERANCE){
        verbose("Already at position\n");
        DBG("Already at position");
        return 0;
    }
    if(chkMove(rawspeed)) return 1;
    unsigned char buf[6] = {0,};
    DBG("Start moving with speed %d, target position: %lu", REVMIN(rawspeed), targposition);
    buf[1] = CW_ENABLE;
    targspd = rawspeed;
    if(MOTOR_REVERSE) rawspeed = -rawspeed;
    buf[2] = (rawspeed >> 8) & 0xff;
    buf[3] = rawspeed & 0xff;
    DBG("\tBUF: %d, %d, %d, %d", buf[0], buf[1], buf[2], buf[3]);
 //unsigned char obuf[8];
    if(can_send_chk(buf, NULL)){
        WARNX("Can't move motor!");
        stop();
        return 1;
    }
    //DBG("\tOBUF: %d, %d, %d, %d, %d, %d", obuf[0], obuf[1], obuf[2], obuf[3], obuf[4], obuf[5]);
    double t0 = can_dtime();
    // Steps after stopping = -27.96 + 9.20e-2*v + 3.79e-4*v^2, v in rev/min
    // in rawspeed = -27.96 + 1.84e-2*v + 1.52e-5*v^2
    double rs = fabs((double)rawspeed);
    //double cv = (-27.96 + (1.84e-2 + 1.52e-5*rs)*rs);
    double cv = (CORR0 + (CORR1 + CORR2 * rs)*rs);
    DBG("Corr coefficients: %g, %g, %g; raw speed: %g", CORR0, CORR1, CORR2, rs);
    long corrvalue = (long) cv; // correction due to stopping ramp
    if(corrvalue < 10) corrvalue = 10;
    DBG("start-> curpos: %ld, difference: %ld, corrval: %ld",
        curposition, olddiffr, corrvalue);
    int errctr = 0, passctr = 0;
    while(can_dtime() - t0 < MOVING_TIMEOUT){
        double speed;
        if(emerg_stop){ // emergency stop activated
            WARNX("Activated stop while moving");
            stop();
            curstatus = STAT_OK;
            return 1;
        }
        if(get_motor_speed(&speed) != CAN_NOERR){ // WTF?
            WARNX("Unknown situation: can't get speed of moving motor");
            stop();
            curstatus = STAT_ERROR;
            return 1;
        }
        //getPos(NULL);
        if(chkMove(targspd)){
            WARNX("Can't move further!");
            stop();
            return 1;
        }
        if(fabs(speed) < 0.1){
            if(can_dtime() - t0 > TACCEL){
                WARNX("Motor can't moving! Time after start=%.3fs.", can_dtime()-t0);
                curstatus = STAT_ERROR;
                stop();
                return 1;
            }
        }
        if(!getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition)) continue;
        long diffr = labs((long)targposition - (long)curposition);
        DBG("Speed: %g, curpos: %ld, diff: %ld", speed, curposition, diffr);
        if(diffr < corrvalue){
            DBG("OK! almost reach: olddif=%ld, diff=%ld, corrval=%ld, tm=%g", olddiffr, diffr, corrvalue, can_dtime()-t0);
            olddiffr = diffr;
            break;
        }
        if(diffr > olddiffr){ // pass over target -> stop
            if(++passctr > 2) break;
        }
        if(diffr >= olddiffr){ // motor stall -> stop
            ++errctr;
            DBG("errctr: %d", errctr);
            if(errctr > STALL_MAXCTR) break;
        }else errctr = 0;
        olddiffr = diffr;
    }
    if(can_dtime() - t0 > MOVING_TIMEOUT){
        WARNX("Error: timeout, but motor still not @ position! STOP!");
        stop();
        curstatus = STAT_ERROR;
        return 1;
    }
    DBG("end-> curpos: %ld, difference: %ld, tm=%g\n", curposition, targposition - curposition, can_dtime()-t0);
    if(waitTillStop()) return 1;
    if(labs((long)targposition - (long)curposition) > RAWPOS_TOLERANCE)
        verbose("Current (%ld) position is too far from target (%ld)\n", curposition, targposition);
    DBG("stop-> curpos: %ld, difference: %ld, tm=%g\n", curposition, targposition - curposition, can_dtime()-t0);
    curstatus = STAT_OK;
    return 0;
}

/**
 * @brief move2pos - accurate focus moving to target position (in encoder's units)
 * @param target   - position 2 move (in mm)
 * @return 0 if all OK
 */
int move2pos(double target){
    if(!motorRDY || !encoderRDY) return 1;
    FNAME();
    double cur;
    if(getPos(&cur)){
        WARNX("Can't get current position!");
        return 1;
    }
    unsigned long targposition = FOC_MM2RAW(target);
    DBG("Raw target position: %lu", targposition);
    if(target > FOCMAX_MM || target < FOCMIN_MM){
        WARNX("Target focus position over the available range!");
        return 1;
    }
    if(labs((long)targposition - (long)curposition) < RAWPOS_TOLERANCE){
        verbose("Already at position\n");
        return 0;
    }
    long spd, targ0pos = (long)targposition - (long)dF0, absdiff = labs(targ0pos - (long)curposition),
            sign = (targ0pos > (long)curposition) ? 1 : -1;
    DBG("absdiff: %ld", absdiff);
    if(absdiff > ENCODER_DIFF_SPEED1) spd = sign*MAXSPEED;
    else if(absdiff > ENCODER_DIFF_SPEED2) spd = sign*MAXSPEED / 2;
    else if(absdiff > ENCODER_DIFF_SPEED3) spd = sign*MAXSPEED / 3;
    else spd = sign*MINSPEED;
    int16_t targspd = (int16_t) spd;
    DBG("TARGSPD: %d", targspd);
/*    if(spd > INT16_MAX) targspd = INT16_MAX;
    else if(spd < INT16_MIN) targspd = INT16_MIN;
    fix_targspeed(&targspd);*/
    // check moving direction: thin focus correction always should run to negative!
    if(targposition > curposition){ // we are from the left
        if(targspd > MINSPEED*3/2){ // omit rough moving to focus value if there's too little distance towards target
            // rough moving
            DBG("1) ROUGH move to the RIGHT: curpos=%ld, difference=%ld\n", curposition, targ0pos - (long)curposition);
            if(move(targ0pos, RAWSPEED(targspd))){
                return 1;
            }
        }
    }else{ // we are from the right - move to the point @ left side of target
        DBG("1) ROUGH move to the LEFT: curpos=%ld, difference=%ld\n", curposition, targ0pos - (long)curposition);
        if(move(targ0pos, RAWSPEED(targspd))){
            DBG("Error in move?");
            return 1;
        }
    }
    // now move precisely
    if(!getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition)){
        WARNX("Can't get current position");
        return 1;
    }
    if(labs((long)targposition - (long)curposition) < RAWPOS_TOLERANCE){
        verbose("Catch the position @ rough moving\n");
        DBG("Catch the position @ rough moving");
        return 0;
    }
    if(curposition > targposition){ // we should be from the left of target
        WARNX("Error in current position: %.3f instead of %.3f!", FOC_RAW2MM(curposition), FOC_RAW2MM(targposition));
        return 1;
    }
    DBG("2) curpos: %ld, difference: %ld\n", curposition, (long)targposition - (long)curposition);
    // now make an accurate moving
    if(move(targposition, RAWSPEED(MINSPEED))){
        WARNX("Can't catch focus precisely!");
        return 1;
    }
    if(!getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition)){
        WARNX("Can't get current position");
        return 1;
    }
    if(labs((long)targposition - (long)curposition) > RAWPOS_TOLERANCE){
        verbose("Stopped over the accuracy range\n");
        return 1;
    }
    return 0;
}

// return 0 if all OK
int get_pos_speed(unsigned long *pos, double *speed){
    FNAME();
    int ret = 0;
    if(pos){
        if(!encoderRDY) *pos = FOC_MM2RAW(3.);
        else if(!getLong(encnodenum, DS406_POSITION_VAL, 0, pos)) ret = 1;
    }
    if(speed){
        if(!motorRDY){
            *speed = 0.;
            return ret;
        }
        if(get_motor_speed(speed) != CAN_NOERR){
            *speed = 0.;
            ret = 1;
        }else if(MOTOR_REVERSE) *speed = -*speed;
    }
    return ret;
}

void movewithmon(double spd){
    if(!motorRDY || !encoderRDY) return;
    unsigned char buf[6] = {0,};
    if(fabs(spd) < MINSPEED || fabs(spd) > MAXSPEED){
        WARNX("Target speed should be be from %d to %d (rev/min)", MINSPEED, MAXSPEED);
        return;
    }
    unsigned long pos, oldpos = 0, startpos;
    double speed;
    get_pos_speed(&startpos, NULL); // starting position
    int16_t targspd = RAWSPEED(spd);
    buf[1] = CW_ENABLE;
    if(MOTOR_REVERSE) targspd = -targspd;
    buf[2] = (targspd >> 8) & 0xff;
    buf[3] = targspd & 0xff;
    if(can_send_chk(buf, NULL)){
        WARNX("Can't move motor!");
        return;
    }
    double t0 = can_dtime(), tlast = t0;
    green("\nAcceleration with monitoring not longer than for 4 seconds\n\n");
#define PRINT()  do{printf("t=%g, pos=%lu (%.3fmm), spd=%g, Dpos=%.0f\n", tcur - t0, pos, \
    FOC_RAW2MM(pos), speed, oldpos ? ((double)pos - oldpos)/(tcur - tlast) : 0);}while(0)
    while(can_dtime() - t0 < 4.){
        if(get_pos_speed(&pos, &speed)){ // can't get speed? WTF?
            WARNX("Strange things are going here...");
            break;
        }
        double tcur = can_dtime();
        PRINT();
        oldpos = pos;
        if(fabs(speed - spd) < 1.){
            green("\tTarget speed reached for %.2fs, DPOS=%ld\n", tlast - t0, pos - startpos);
            break;
        }
        tlast = tcur;
        can_dsleep(0.03);
    }
    green("\nMove for 3 seconds with constant speed\n\n");
    get_pos_speed(&startpos, NULL);
    t0 = can_dtime();
    do{
        can_dsleep(0.5);
        if(get_pos_speed(&pos, &speed)){ // can't get speed? WTF?
            WARNX("Strange things are going there...");
            break;
        }
        double tcur = can_dtime();
        PRINT();
        oldpos = pos;
        tlast = tcur;
    }while(can_dtime() - t0 < 3.);
    double meanspd = ((double)pos - startpos) / (tlast - t0);
    green("\tMean pos speed: %.0f (%g mm/s)\n", meanspd, FOC_RAW2MM(meanspd));
    green("\nStop with monitoring not longer than for 4 seconds\n\n");
    get_pos_speed(&startpos, NULL);
    t0 = can_dtime();
    for(int i = 0; i < 100 && stop(); ++i);
    while(can_dtime() - t0 < 4.){
        get_pos_speed(&pos, NULL);
        double tcur = can_dtime();
        PRINT();
        if(oldpos == pos){
            green("\tStopped for %.2fs, DPOS=%ld\n", tlast - t0, pos - startpos);
            break;
        }
        oldpos = pos;
        tlast = tcur;
        can_dsleep(0.03);
    }
#undef PRINT
}

sysstatus get_status(){
    return curstatus;
}
