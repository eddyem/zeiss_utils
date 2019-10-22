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

// amount of encoder's counts when stop = rawspeed^2/STOPPING_COEFF
#define STOPPING_COEFF      (117500L)

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
#define MAXSPEED            1200
#define MINSPEED            130
// moving timeout (*0.01s)
#define MOVING_TIMEOUT      5000

// raw position precision
#define RAWPOS_TOLERANCE    20
// raw dF value for accurate focussing
#define dF0                 100
// minimal & maximal focus positions (should be >min+dF0 & <max-dF0)
#define FOCMIN              200
#define FOCMAX              320000
// the same in mm
#define FOCMIN_MM           0.1
#define FOCMAX_MM           76.0
// permitted distance to end-switch
#define ESW_DIST_ALLOW      1.0

// constants for focus conversion: foc_mm = (foc_raw - FOCRAW_MM0) / FOCSCALE_MM
#define FOCSCALE_MM         4096.
#define FOCRAW_MM0          0
#define FOC_RAW2MM(x)       ((x-FOCRAW_MM0)/FOCSCALE_MM)
#define FOC_MM2RAW(x)       (FOCRAW_MM0+x*FOCSCALE_MM)

#endif // HW_DEPENDENT__
