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

#pragma once
#ifndef CAN_ENCODER_H__
#define CAN_ENCODER_H__

#include <stdint.h>

typedef enum{
    CAN_NOERR = 0,
    CAN_CANTSEND,
    CAN_NOANSWER,
    CAN_WARNING,
    CAN_ERROR
} canstatus;

typedef enum{
    ESW_INACTIVE = 0,
    ESW_CW_ACTIVE = 1,
    ESW_CCW_ACTIVE = 2,
    ESW_BOTH_ACTIVE = 3
} eswstate;

// system state
typedef enum{
    // non-blocking statuses:
    STAT_OK,        // all OK
    // blocking statuses:
    STAT_ESW,       // end-switch active
    STAT_GOFROMESW, // mowing from end-switch
    STAT_ERROR,     // error state
    STAT_FORBIDDEN, // forbidden position
    STAT_DAMAGE     // the device in damaged state and can't work further
} sysstatus;

int init_encoder(int encnode, int reset);
void returnPreOper(long long presetval);
int getPos(double *pos);
double curPos();
int init_motor_ids(int addr);
void movewithmon(double spd);
canstatus get_motor_speed(double *spd);
canstatus get_endswitches(eswstate *Esw);
int move2pos(double target);
int stop();
int movewconstspeed(int16_t spd);
int go_out_from_ESW();
sysstatus get_status();
int get_pos_speed(unsigned long *pos, double *speed);

#endif // CAN_ENCODER_H__
