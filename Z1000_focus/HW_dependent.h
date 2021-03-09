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
#ifndef HW_DEPENDENT__
#define HW_DEPENDENT__

#include "motor_cancodes.h"

// On lowest speeds Taccelerated = 0.22s, so wait no more than 0.25s when motor starts
#define TACCEL              (0.25)
// max amount of cycles when motor stalled
#define STALL_MAXCTR        (50)

// direction of motor rotation positive to encoder (1 - negative)
#define MOTOR_REVERSE       (0)

// End switches (lowest bit - DI00)
// clockwise: DI4
#define ESW_CW              (1<<4)
// counterclockwise: DI5
#define ESW_CCW             (1<<5)
// parameter indexes of esw:
#define PAR_CW_IDX          PAR_DI04_IDX
#define PAR_CCW_IDX         PAR_DI05_IDX

// constants
// translate raw values to revolutions per minute
#define REVMIN(x)           (x/5)
// rev/min to raw speed value
#define RAWSPEED(x)         (x*5)
// max/min speed (rev/min)
#define MAXSPEED            (1200)
#define MINSPEED            (350)
// encoder differences (if larger) for speed (MAXSPEED, MAXSPEED/2 and MAXSPEED/3) select
#define ENCODER_DIFF_SPEED1 (1500)
#define ENCODER_DIFF_SPEED2 (500)
#define ENCODER_DIFF_SPEED3 (150)
// speed to move from ESW
#define ESWSPEED            (350)
// moving timeout: 5minutes
#define MOVING_TIMEOUT      (300)
// correction parameters: steps after stopping = CORR0 + (CORR1 + CORR2*rs)*rs)
// where rs is raw speed
#define CORR0               (-46.0)
#define CORR1               (4.2857e-3)
#define CORR2               (1.5714e-5)


// constants for focus conversion: foc_mm = (foc_raw - FOCRAW_0) / FOCSCALE_MM
#define FOCSCALE_MM         (4096.)
#define FOCRAW_0            (15963187.)
#define FOC_RAW2MM(x)       (((x)-FOCRAW_0)/FOCSCALE_MM)
#define FOC_MM2RAW(x)       (FOCRAW_0+(x)*FOCSCALE_MM)

// raw position precision - 2.5um
#define RAWPOS_TOLERANCE    10
// raw dF value for accurate focussing (rough move to F-dF0 and after this slow move to F)
#define dF0                 250
// minimal & maximal focus positions (should be >min+dF0 & <max-dF0)
#define FOCMIN_MM           2.75
#define FOCMAX_MM           76.
// -//- raw values
#define FOCMIN              FOC_MM2RAW(FOCMIN_MM)
#define FOCMAX              FOC_MM2RAW(FOCMAX_MM)
// permitted distance to end-switch
#define ESW_DIST_ALLOW      1.0
// focus raw positions @ endswitches (< or >)
#define FOCPOS_CW_ESW       FOC_MM2RAW((FOCMAX_MM - ESW_DIST_ALLOW))
#define FOCPOS_CCW_ESW      FOC_MM2RAW((FOCMIN_MM + ESW_DIST_ALLOW))
#endif // HW_DEPENDENT__
