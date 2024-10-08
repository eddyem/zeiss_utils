/**
  Commands & other data for absolute rotary encoder DS406
 **/
#pragma once
#ifndef DS406_CANOPEN_H__
#define DS406_CANOPEN_H__

#define DS406_DEVTYPE           0x1000
#define DS406_ERRORREG          0x1001
#define DS406_MANUFSTATUS       0x1002
#define DS406_COBID_SYNC        0x1005
#define DS406_MANDEVNAME        0x1008
#define DS406_MANHW_VERS        0x1009
#define DS406_MANSW_VERS        0x100A
#define DS406_STORE_PARAMS      0x1010
#define DS406_RESTORE_DEF       0x1011
#define DS406_COBID_EMERG       0x1014
#define DS406_INHIB_TM_EMERG    0x1015
#define DS406_PROD_HEARTB_TM    0x1017
#define DS406_IDENT_OBJ         0x1018
#define DS406_ERROR_BEHAVIOUR   0x1029

#define DS406_PDO1              0x1800
#define DS406_PDO2              0x1801
#define DS406_PDO1_MAPPED       0x1A00
#define DS406_PDO2_MAPPED       0x1A01
#define DS406_PDO3_MAPPED       0x1A02
#define DS406_PDO4_MAPPED       0x1A03

#define DS406_BAUDRATE          0x2100
#define DS406_NODE_NUMBER       0x2101
#define DS406_TERMINATOR        0x2102
#define DS406_NMT_AUTOSTART     0x2104
#define DS406_PDO_TRIG_THRES    0x2105
#define DS406_FILTER_CONFIG     0x2106
#define DS406_CUSTOMER_MEMRY    0x2110
#define DS406_SENSOR_AMPLITUDE  0x2200
#define DS406_TARG_FREQ_DEVIAT  0x2201

//  Configuration parameters
// subindex=1 - Safety code sequence (uint16) - 0 for CW, 1 for CCW
// 2 - Safety preset value (uint32)
// 3 - Inverted safety preset value (uint32)
#define DS406_CONF_PARAMETERS   0x5000
#define DS406_CONF_VALID        0x50FE
#define DS406_CONF_VALID_VALID  0xA5
#define DS406_CONF_CHECKSUM     0x50FF


#define DS406_OPER_PARAMS       0x6000
#define DS406_MEAS_UNITS_PERREV 0x6001
#define DS406_TOT_MEAS_RANGE    0x6002
#define DS406_PRESET_VAL        0x6003
#define DS406_POSITION_VAL      0x6004
#define DS406_PDO_IS_POSITION   0x60040020
#define DS406_POS_RAW_VAL       0x600C
#define DS406_SPEED_VAL         0x6030
#define DS406_PDO_IS_SPEED      0x60300110
#define DS406_CYCLE_TIMER       0x6200
#define DS406_WORK_AREA_STATE   0x6400
#define DS406_WORK_AREA_LIM_LO  0x6401
#define DS406_WORK_AREA_LIM_HI  0x6401
#define DS406_OPER_STATUS       0x6500
#define DS406_TURN_RESOLUT      0x6501
#define DS406_REVOL_NUMBER      0x6502
#define DS406_ALARMS            0x6503
#define DS406_SUPP_ALARMS       0x6504
#define DS406_WARNINGS          0x6505
#define DS406_SUPP_WARNINGS     0x6506
#define DS406_PROF_SW_VERS      0x6507
#define DS406_OFFSET_VAL        0x6509
#define DS406_MODULE_ID         0x650A
#define DS406_SERIAL_NUMBER     0x650B




#endif // DS406_CANOPEN_H__
