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

#include <string.h> // memcpy
#include <math.h>   // fabs
#include "canopen.h"
#include "can_encoder.h"
#include "usefull_macros.h"
#include "DS406_canopen.h"
#include "motor_cancodes.h"
#include "HW_dependent.h"
#include "socket.h"

// printf when -v
extern int verbose(const char *fmt, ...);

// CAN bus IDs: for motor's functions (PI ID [F=4] == PO ID[F=3] + 1) and parameters
static unsigned long motor_id = 0, motor_p_id = 0;//, bcast_id = 1;
// current motor position
static unsigned long curposition = 0;
// encoder's node number
static int encnodenum = 0;

static canstatus can_write_par(uint8_t subidx, uint16_t idx, uint32_t *parval);
static canstatus can_read_par(uint8_t subidx, uint16_t idx, uint32_t *parval);
static int move(unsigned long targposition, int16_t rawspeed);

/**
 * @brief init_encoder - encoder's interface initialisation
 * @param encnode - encoder's node number
 * @param reset   - reset node before operating
 * @return 0 if all OK
 */
int init_encoder(int encnode, int reset){
    unsigned long lval;
    encnodenum = encnode;
    verbose("cur node: %d\n", encnodenum);
    if(!initNode(encnodenum)){
        ERRX("Can't find device with Node address %d!", encnodenum);
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
/*
    //setByte(encnodenum, 0x3010, 1, 1); // Posital's encoders specific! (Could not be saved!)
    if(G->verbose){
        if(getLong(encnodenum, DS406_TURN_RESOLUT, 0, &lval)) printf("TURN_RESOLUT: %08lx (%ld)\n",lval, lval);
        if(getShort(encnodenum, DS406_REVOL_NUMBER, 0, &sval)) printf("REVOL_NUMBER: %04x (%d)\n",sval, sval);
        if(getLong(encnodenum, DS406_MODULE_ID, 1, &lval)) printf("OFFSET VALUE: %08lx (%ld)\n",lval, lval);
        if(getLong(encnodenum, DS406_MODULE_ID, 2, &lval)) printf("MIN POS: %08lx (%ld)\n",lval, lval);
        if(getLong(encnodenum, DS406_MODULE_ID, 3, &lval)) printf("MAX POS: %08lx (%ld)\n",lval, lval);
    }
*/
    verbose("Set operational... ");
    startNode(encnodenum);
    int n, i = recvNextPDO(0.1, &n, &lval);
    int state = getNodeState(encnodenum);
    verbose("State=%02x\n", state);
    if(state == NodeOperational) verbose("Ok!\n");
    else{
        WARNX("Failed to start node!");
        returnPreOper();
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
    return 0;
}

/**
 * @brief go_out_from_ESW - check ESW roles, check ESW states and try to go out if stay on ESW
 * @return 0 if all OK
 */
int go_out_from_ESW(){
    uint32_t cw, ccw, parval;
    // check esw roles
    if(CAN_NOERR != can_read_par(PAR_DI_SUBIDX, PAR_CW_IDX, &cw)) goto bad;
    if(CAN_NOERR != can_read_par(PAR_DI_SUBIDX, PAR_CCW_IDX, &ccw)) goto bad;
    parval = DI_ENSTOP;
    if(cw != DI_ENSTOP || ccw != DI_ENSTOP){ // activate enable/stop
        if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, PAR_CW_IDX, &parval)) goto bad;
        if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, PAR_CCW_IDX, &parval)) goto bad;
    }
    // check esw
    eswstate e;
    if(CAN_NOERR != get_endswitches(&e)){
        WARNX("Can't read end-switches state");
        return 1;
    }
    if(e == ESW_BOTH_ACTIVE){ // error situation!
        WARNX("Error: both end-switches are active!");
        return 1;
    }
    if(e == ESW_INACTIVE){
        DBG("Esw inactive");
        return 0;
    }
    // try to move from esw
    parval = DI_NOFUNC;
    uint16_t idx = (e == ESW_CW_ACTIVE) ? PAR_CW_IDX : PAR_CCW_IDX;
    if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, idx, &parval)) goto bad; // turn off motor stopping @esw
    int16_t speed = (e == ESW_CW_ACTIVE) ? -RAWSPEED(MINSPEED) : RAWSPEED(MINSPEED);
    for(int i = 0; i < 5; ++i){ // 5 tries to move out
        getPos(NULL);
        DBG("try %d, pos: %lu, E=%d", i, curposition, e);
        unsigned long targ = (e == ESW_CW_ACTIVE) ? curposition - (double)FOCSCALE_MM*0.2 : curposition + (double)FOCSCALE_MM*0.2;

        if(move(targ, speed)) continue;
        get_endswitches(&e);
        if(e == ESW_INACTIVE) break;
    }
    // return esw state
    parval = DI_ENSTOP;
    if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, PAR_CW_IDX,  &parval)) goto bad;
    if(CAN_NOERR != can_write_par(PAR_DI_SUBIDX, PAR_CCW_IDX, &parval)) goto bad;
    if(e != ESW_INACTIVE){
        WARNX("Can't move out of end-switch");
        return 1;
    }
    return 0;
bad:
    WARNX("Can't get/set esw parameters");
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
    int r = !(getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition));
    if(pos) *pos = FOC_RAW2MM(curposition);
    return r;
}

// just return value of curposition in mm
double curPos(){
    return FOC_RAW2MM(curposition);
}

/**
 * @brief returnPreOper - return encoder into pre-operational state
 */
void returnPreOper(){
    verbose("Return to Pre-operational... ");
    setPreOper(encnodenum);
    int state=getNodeState(encnodenum);
    verbose("State=0x%02x - ", state);
    if(state == NodePreOperational){
        verbose("Ok!\n");
    }else{
        verbose("Failed!\n");
    }
}

/**
 * @brief fix_targspeed - fix raw speed value if it is greater MAXSPEED or less than MINSPEED
 * @param targspd (io) - raw target speed
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
    const int l = 6; // frame length
    /*if(G->verbose){
        printf("Send frame to ID=%lu: ", motor_id);
        for(int i = 0; i < l; ++i)
            printf(" %02x", buf[i]);
        printf("\n");
    }*/
    if(can_send_frame(motor_id, l, buf) <= 0){
        WARNX("Error sending CAN frame (len %d)", l);
        return CAN_CANTSEND;
    }
    int I, rxpnt, idr, dlen;
    double rxtime;
    unsigned char rdata[8];
    can_clean_recv(&rxpnt, &rxtime);
    for(I = 0; I < 50; ++I){
        if(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata) && (idr&0x1fffffff) == motor_id+1) break;
        can_dsleep(0.01);
    }
    if(I == 50){
        WARNX("Error getting answer");
        return CAN_NOANSWER;
    }
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
    const int l = 8; // frame length
    int I, rxpnt, idr, dlen;
    double rxtime;
    unsigned char rdata[8];
    can_clean_recv(&rxpnt, &rxtime);
    if(can_send_frame(motor_p_id, l, buf) <= 0){
        WARNX("Error sending CAN frame (len %d)", l);
        return CAN_CANTSEND;
    }
    /*
green("Sent param:     ");
for(int i=0; i<l; ++i) printf("0x%02x ", buf[i]);
printf("\n");
*/
    for(I = 0; I < 50; ++I){
        if(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata) && (idr&0x1fffffff) == motor_p_id+1) break;
        can_dsleep(0.01);
    }
    if(I == 50){
        WARNX("Error getting answer");
        return CAN_NOANSWER;
    }
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
    uint8_t buf[8] = {CAN_WRITEPAR_CMD, subidx,0}, obuf[8];
    buf[2] = idx >> 8;
    buf[3] = idx & 0xff;
    if(parval){
        uint32_t par = *parval;
        obuf[4] = (par >> 24) & 0xff;
        obuf[5] = (par >> 16) & 0xff;
        obuf[6] = (par >> 8)  & 0xff;
        obuf[7] = par & 0xff;
    }
    return can_send_param(buf, obuf);
}


/**
 * @brief get_motor_speed
 * @param motstatus (o) - status (if !NULL)
 * @return speed in rev/min
 */
canstatus get_motor_speed(double *spd){
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
    /*
    union{
        int16_t i16;
        uint8_t c8[2];
    } mspd;
    mspd.c8[0] = rdata[3];
    mspd.c8[1] = rdata[2];
    */
    return CAN_NOERR;
}

/**
 * @brief get_endswitches - get state of end-switches
 * @param Esw (o) - end-switches state
 * @return
 */
canstatus get_endswitches(eswstate *Esw){
    uint32_t val = 0;
    canstatus s = can_read_par(PAR_DI_SUBIDX, PAR_DIST_IDX, &val);
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
    unsigned char buf[6] = {0, CW_STOP,0,};
    if(can_send_chk(buf, NULL)){
        WARNX("Can't stop motor!");
        return 1;
    }
    return 0;
}

/**
 * @brief movewconstspeed - move with constant speed
 * @param rawspd - given speed
 * @return 0 if all OK
 */
int movewconstspeed(int spd){
    int16_t targspd = RAWSPEED(spd);
    unsigned char buf[8] = {0,};
    buf[1] = CW_ENABLE;
    if(MOTOR_REVERSE) spd = -spd;
    buf[2] = (targspd >> 8) & 0xff;
    buf[3] = targspd & 0xff;
    if(can_send_chk(buf, NULL)){
        WARNX("Can't move motor!");
        return 1;
    }
    return 0;
}

/**
 * @brief move - move focuser from current position to approximately `targposition` with speed `rawspeed`
 * @param targposition - target position in raw value
 * @param rawspeed - speed in raw value
 * @return 0 if all OK
 */
static int move(unsigned long targposition, int16_t rawspeed){
    if(abs(targposition - curposition) < RAWPOS_TOLERANCE){
        verbose("Already at position\n");
        return 0;
    }
    unsigned char buf[6] = {0,};
    DBG("Start moving with speed %d", rawspeed);
    buf[1] = CW_ENABLE;
    if(MOTOR_REVERSE) rawspeed = -rawspeed;
    buf[2] = (rawspeed >> 8) & 0xff;
    buf[3] = rawspeed & 0xff;
    DBG("\tBUF: %d, %d, %d, %d", buf[0], buf[1], buf[2], buf[3]);
    if(can_send_chk(buf, NULL)){
        WARNX("Can't move motor!");
        stop();
        return 1;
    }
#ifdef EBUG
    double t0 = can_dtime();
#endif
    long olddiffr = targposition - curposition;
    long corrvalue = (long)rawspeed*(long)rawspeed / STOPPING_COEFF; // correction due to stopping ramp
    DBG("start-> curpos: %ld, difference: %ld, corrval: %ld, tm=%g",
        curposition, olddiffr, corrvalue, can_dtime()-t0);
    int i, zerctr = 0;
    for(i = 0; i < MOVING_TIMEOUT; ++i){
        can_dsleep(0.01);
        //uint16_t motstat;
        double speed;
        eswstate esw;
        if(emerg_stop){ // emergency stop activated
            WARNX("Activated stop while moving");
            stop();
            return 1;
        }
        if(get_motor_speed(&speed) != CAN_NOERR){ // WTF?
            WARNX("Unknown situation: can't get speed of moving motor");
            stop();
            return 1;
        }
        if(get_endswitches(&esw) != CAN_NOERR){
            WARNX("Can't get endswitches state, stopping");
            stop();
            return 1;
        }
        if(esw != ESW_INACTIVE){ // OOps, something wrong
            if(esw == ESW_BOTH_ACTIVE){ // error!
                WARNX("Check end-switches, both active!");
                stop();
                return 1;
            }
//TODO: check wrong end-switches!
            if(rawspeed > 0){ // moving CW, don't care for CCW esw state
                if(esw == ESW_CW_ACTIVE){
                    WARNX("CW mowing: end-switch!");
                    stop();
                    return 1;
                }
            }else{ // movig CCW
                if(esw == ESW_CCW_ACTIVE){
                    WARNX("CCW mowing: end-switch!");
                    stop();
                    return 1;
                }
            }
        }
        if(fabs(speed) < 0.1){ // || (motstat & (SW_B_UNBLOCK|SW_B_READY|SW_B_POUNBLOCK)) != (SW_B_UNBLOCK|SW_B_READY|SW_B_POUNBLOCK)){
            if(++zerctr == 10){
                WARNX("Motor stopped while moving!");
                stop();
                return 1;
            }
        }else zerctr = 0;
        if(!getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition)) continue;
        long diffr = targposition - curposition;
        //DBG("Speed: %g, curpos: %ld, diff: %ld", speed, curposition, diffr);
        if(abs(diffr) < corrvalue || abs(diffr) - RAWPOS_TOLERANCE > abs(olddiffr)){
            DBG("OK! almost reach: olddif=%ld, diff=%ld, corrval=%ld, tm=%g", olddiffr, diffr, corrvalue, can_dtime()-t0);
            olddiffr = diffr;
            break;
        }
        olddiffr = diffr;
    }
    if(i == MOVING_TIMEOUT){
        WARNX("Error: timeout, but motor still not @ position! STOP!");
        stop();
        return 1;
    }
    DBG("end-> curpos: %ld, difference: %ld, tm=%g\n", curposition, targposition - curposition, can_dtime()-t0);
    long oldposition;
    int r = stop();
    if(r) r = stop();
    if(!r) do{ // wait till stop
        oldposition = curposition;
        can_dsleep(0.1);
        getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition);
        //DBG("wait-> curpos: %ld, difference: %ld, tm=%g\n", curposition, targposition - curposition, can_dtime()-t0);
    }while((long)curposition != oldposition);
    if(abs(targposition - curposition) > RAWPOS_TOLERANCE)
        verbose("Current (%ld) position is too far from target (%ld)\n", curposition, targposition);
    DBG("stop-> curpos: %ld, difference: %ld, tm=%g\n", curposition, targposition - curposition, can_dtime()-t0);
    return r;
}

/**
 * @brief move2pos - accurate focus moving to target position (in encoder's units)
 * @param target   - position 2 move (in mm)
 * @return 0 if all OK
 */
int move2pos(double target){
    double cur;
    if(getPos(&cur)){
        WARNX("Can't get current position!");
        return 1;
    }
    unsigned long targposition = FOC_MM2RAW(target);
    DBG("Raw target position: %lu", targposition);
    if(target > FOCMAX_MM || target < FOCMIN_MM || targposition < FOCMIN || targposition > FOCMAX){
        WARNX("Target focus position over the available range!");
        return 1;
    }
    if(abs(targposition - curposition) < RAWPOS_TOLERANCE){
        verbose("Already at position\n");
        return 0;
    }
    long targ0pos = (long)targposition + (long)dF0;
    long spd = (targ0pos - (long)curposition) / 2L;
    int16_t targspd = (int16_t) spd;
    if(spd > INT16_MAX) targspd = INT16_MAX;
    else if(spd < INT16_MIN) targspd = INT16_MIN;
    fix_targspeed(&targspd);
    // check moving direction: thin focus correction always should run to negative!
    if(targposition < curposition){ // we are from the right
        if(targspd < -MINSPEED*3/2){ // omit rough moving to focus value if there's too little distance towards target
            // rough moving
            DBG("1) ROUGH move to the LEFT: curpos=%ld, difference=%ld\n", curposition, targ0pos - (long)curposition);
            if(move(targ0pos, RAWSPEED(targspd))){
                return 1;
            }
        }
    }else{ // we are from the left - move to the point @ right side of target
        DBG("1) ROUGH move to the RIGHT: curpos=%ld, difference=%ld\n", curposition, targ0pos - (long)curposition);
        if(move(targ0pos, RAWSPEED(targspd))){
            return 1;
        }
    }
    // now move precisely
    if(!getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition)){
        WARNX("Can't get current position");
        return 1;
    }
    if(abs(targposition - curposition) < RAWPOS_TOLERANCE){
        verbose("Catch the position @ rough movint\n");
        return 0;
    }
    if(curposition < targposition){
        WARNX("Error in current position!");
        return 1;
    }
    DBG("2) curpos: %ld, difference: %ld\n", curposition, (long)targposition - (long)curposition);
    // now make an accurate moving
    if(move(targposition, -(RAWSPEED(MINSPEED)))){
        WARNX("Can't catch focus precisely!");
        return 1;
    }
    if(!getLong(encnodenum, DS406_POSITION_VAL, 0, &curposition)){
        WARNX("Can't get current position");
        return 1;
    }
    if(abs(targposition - curposition) > RAWPOS_TOLERANCE){
        verbose("Stopped over the accuracy range\n");
        return 1;
    }
    return stop();
}

// return 0 if all OK
static int get_pos_speed(unsigned long *pos, double *speed){
    int ret = 0;
    if(pos){
        if(!getLong(encnodenum, DS406_POSITION_VAL, 0, pos)) ret = 1;
    }
    if(speed){
        if(get_motor_speed(speed) != CAN_NOERR){
            *speed = 0.;
            ret = 1;
        }else if(MOTOR_REVERSE) *speed = -*speed;
    }
    return ret;
}

void movewithmon(double spd){
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
