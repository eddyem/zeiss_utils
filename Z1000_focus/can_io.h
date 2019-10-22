// (c) vsher@sao.ru

#pragma once

#ifndef CAN_IO_H__
#define CAN_IO_H__

#define _U_    __attribute__((__unused__))


#define CAN_CTLR_SIZE 1024       /* size of client process shared area */
#define CAN_RX_SIZE   1000       /* max. # frames in Rx-buffer */
#define CAN_RTR_FLAG  0x20000000 /* send frame as Remote Transmission Request */
#define CAN_EXT_FLAG  0x40000000 /* send frame with extended 29-bit ID, 11-bit otherwise */

int can_wait(int fd, double tout);
#define can_delay(Tout) can_wait(0, Tout)
void set_server_mode(int mode);
int can_server();
void set_sending_mode(int to_server);
int can_sending_mode();
int can_card();
int can_gate();
double can_gate_time_offset();
void setup_can_net(unsigned long ipaddr, int port, unsigned long acckey);
unsigned long get_acckey();
void *init_can_io();
void *start_can_io(void *arg);
void can_put_buff_frame(double rtime, int id, int length, unsigned char data[]);
int can_io_ok();
int can_io_shm_ok();
int can_ok();
void can_clean_recv(int *pbuf, double *rtime);
int can_get_buff_frame(int *pbuf, double *rtime,
          int *id, int *length, unsigned char data[]);
int can_recv_frame(int *pbuf, double *rtime,
          int *id, int *length, unsigned char data[]);
int can_send_frame(unsigned long id, int length, unsigned char data[]);
void can_exit(int sig);
char *time2asc(double t);
double can_dsleep(double dt);
double can_dtime();
char *can_atime();
void can_prtime(FILE *fd);
#endif // CAN_IO_H__
