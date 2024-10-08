/* CAN I/O library (for compatibility with the old one,      */
/*                  but through the new SocketCAN interface) */

#include <linux/can.h>

#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG  0x40000000 /* frame as Remote Transmission Request */
#endif
#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG  0x80000000 /* frame with extended 29-bit ID, 11-bit otherwise */
#endif
#define CAN_EXT_FLAG  CAN_EFF_FLAG

int can_wait(int fd, double tout);
#define can_delay(Tout) can_wait(0, Tout)
void *init_can_io();
int can_ok();
#define can_io_ok()  can_ok()
void can_clean_recv(int *psock, double *rtime);
int can_recv_frame(int *psock, double *rtime,
		  canid_t *id, int *length, unsigned char data[]);
int can_send_frame(canid_t id, int length, unsigned char data[]);
void can_exit(int sig);
char *time2asc(double t);
double can_dsleep(double dt);
double can_dtime();
void can_prtime(FILE *fd);
void set_sending_mode(int);
int can_sending_mode();
