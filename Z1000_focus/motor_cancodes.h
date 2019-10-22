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
#ifndef MOTOR_CANCODES_H__
#define MOTOR_CANCODES_H__

// functions PO/PI/etc
#define PO_FNO      3
#define PI_FNO      4
#define SYNC_FNO    5
#define GR_FNO      6
#define PARAMDATA   512

// calculation of ID from address
#define MOTOR_PO_ID(addr)           ((addr<<3) + PO_FNO)
#define MOTOR_PAR_ID(addr)          (PARAMDATA + (addr<<3) + PO_FNO)
//#define MOTOR_BCAST_PO_ID(addr)     ((addr<<3) + GR_FNO)

// zero's (command) byte of parameter request
// read & write parameters
#define CAN_READPAR_CMD     0x31
#define CAN_WRITEPAR_CMD    0x32
// error flag in answer
#define CAN_PAR_ERRFLAG     0x80

// control word bits & bit combination
#define CW_RAPIDSTOP    0
#define CW_INHIBIT      1
#define CW_STOP         2
#define CW_ENABLE       6
#define CW_B_BLOCK      (1<<0)
#define CW_B_ENRAPID    (1<<1)
#define CW_B_ENSTOP     (1<<2)
#define CW_B_TEMPO      (1<<4)
#define CW_B_PARSET     (1<<5)
#define CW_B_CLERR      (1<<6)

// status word
#define SW_B_UNBLOCK    (1<<0)
#define SW_B_READY      (1<<1)
#define SW_B_POUNBLOCK  (1<<2)
#define SW_B_TEMPO21    (1<<3)
#define SW_B_PARAM21    (1<<4)
#define SW_B_MAILFUN    (1<<5)
// artifical status bit (instead of reserved) for error getting speed
#define SW_B_CANTGETSPD (1<<6)
#define SW_ENABLE       4
#define SW_NOTENABLE    2
#define SW_INHIBIT      1

// state codes (when SW_B_MAILFUN==0)
#define STATE_NOTREADY  0
#define STATE_BLOCK     1
#define STATE_NOPERMIT  2
#define STATE_DETENT    3
#define STATE_PERMIS    4
#define STATE_REGUL     5
#define STATE_FACTORYST 8
#define STATE_CAPTURE   13
#define STATE_W4DATA    16
#define STATE_SAFESTOP  17

// Some parameters: indexes & subindexes
// Digital inputs
#define PAR_DI_SUBIDX   0
// inputs state (lowest bit is DI00)
#define PAR_DIST_IDX    8334
// Speed & current
#define PAR_SPD_SUBIDX  0
#define PAR_CRNT_SUBIDX 0
#define PAR_SPD_IDX     8318
#define PAR_CRNT_IDX    8326
// inputs role
#define PAR_DI00_IDX    8844
#define PAR_DI02_IDX    8336
#define PAR_DI03_IDX    8337
#define PAR_DI04_IDX    8338
#define PAR_DI05_IDX    8339
// roles:
#define DI_NOFUNC       0
#define DI_ENSTOP       1

#endif // MOTOR_CANCODES_H__
