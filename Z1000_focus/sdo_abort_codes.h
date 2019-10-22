// (c) vsher@sao.ru
#pragma once
#ifndef SDO_ABORT_CODES_H__
#define SDO_ABORT_CODES_H__

/* CANopen SDO response Abort codes*/
#define SDO_MAX_ERR 30
struct SDO_Abort_Codes {
   unsigned long code;
   char *text;
} sdo_error[SDO_MAX_ERR] = {
 {0x05030000, "Toggle bit not alternated"},
 {0x05040000, "SDO protocol timed out"},
 {0x05040001, "Client/server command specifier not valid or unknown"},
 {0x05040002, "Invalid block size (block mode only)"},
 {0x05040003, "Invalid sequence number (block mode only)"},
 {0x05040004, "CRC error (block mode only)"},
 {0x05040005, "Out of memory"},
 {0x06010000, "Unsupported access to an object"},
 {0x06010001, "Attempt to read a write only object"},
 {0x06010002, "Attempt to write a read only object"},
 {0x06020000, "Object does not exist in the object dictionary"},
 {0x06040041, "Object cannot be mapped to the PDO"},
 {0x06040042, "The number and length of the objects to be mapped whould exeed PDO length"},
 {0x06040043, "General parameter incompatibility reason"},
 {0x06040047, "General internal incompatibility in the device"},
 {0x06060000, "Access failed due to a hardware error"},
 {0x06070010, "Data type does not match, length of service parameter does not match"},
 {0x06070012, "Data type does not match, length of service parameter too hight"},
 {0x06070013, "Data type does not match, length of service parameter too low"},
 {0x06090011, "Sub-index does not exist."},
 {0x06090030, "Value range of parameter exceeded (only for write access)"},
 {0x06090031, "Value of parameter written too hight"},
 {0x06090032, "Value of parameter written too low"},
 {0x06090036, "Maximum value is less than minimum value"},
 {0x08000000, "General error"},
 {0x08000020, "Data cannot be transferred or stored to the application"},
 {0x08000021, "Data cannot be transferred or stored to the application because of local control"},
 {0x08000022, "Data cannot be transferred or stored to the application because ofthe present device state"},
 {0x08000023, "Object dictionary dynamic generation fails or no object dictionary is present."},
 {0,NULL}
};
static inline char *sdo_abort_text(unsigned long code) {
   int i;
   for(i=0; i<SDO_MAX_ERR && sdo_error[i].text!=NULL; i++)
      if(code==sdo_error[i].code) return sdo_error[i].text;
   return "SDO error of unknown type";
}

#endif // SDO_ABORT_CODES_H__
