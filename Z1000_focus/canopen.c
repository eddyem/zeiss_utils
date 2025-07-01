// (c) vsher@sao.ru
#include "canopen.h"
#include "sdo_abort_codes.h"


static int rxpnt=-1;
static double rxtime=0.;

int sendNMT(int node, int icode){
    unsigned long idt=0;
    int dlen=2;
    unsigned char tdata[2] = {0};
    tdata[0] = icode&0xff;
    tdata[1] = node&0x7f;
    return (can_send_frame(idt, dlen, tdata) > 0);
}

int resetNode2(int oldnode, int newnode){
    int dlen,ntout=50;
    canid_t idr;
    unsigned char rdata[8];
    can_clean_recv(&rxpnt, &rxtime);
    if(!sendNMT(oldnode, 0x81)) return 0;
    for(int i=0; i < ntout; ++i){
        can_dsleep(0.01);
        while(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata)){
            if(idr == (0x700|((canid_t)newnode)) && dlen==1 && rdata[0] == 0) return 1;
        }
    }
    return 0;
}

int resetNode(int node){return resetNode2(node,node);}

int getNodeState(int node){
    /* use Node Guarding Protocol */
    unsigned long idt = (0x700 | (node&0x7f) | CAN_RTR_FLAG);
    int dlen = 0, ntout = 15;
    canid_t idr;
    unsigned char rdata[8];
    can_clean_recv(&rxpnt, &rxtime);
    if(can_send_frame(idt, dlen, rdata)<=0) return 0;
    for(int i=0; i < ntout; ++i){
        can_dsleep(0.01);
        while(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata)) {
            if(idr == (0x700|((canid_t)node)) && dlen == 1) return rdata[0]&0x7f;
        }
    }
    return 0;
}

int initNode(int node){
    int state;
    if(rxpnt<0){
        init_can_io();
        can_clean_recv(&rxpnt, &rxtime);
    }
    if(!can_ok()) return 0;
    if((state = getNodeState(node)) == 0) return 0;
    if(state != NodePreOperational) setPreOper(node);
    return 1;
}

int sendSDOdata(int node, int func, int object, int subindex, unsigned char data[]){
    unsigned long idt = 0x600 | (node&0x7f);
    int dlen = 8;
    unsigned char tdata[8] = {0};
    func &= 0x7f;
    tdata[0] = func;
    tdata[1] = object&0xff;
    tdata[2] = (object&0xff00)>>8;
    tdata[3] = subindex&0xff;
    switch(func){
        case 0x22:
        case 0x23:
        case 0x43: tdata[7]=data[3];
            // FALLTHRU
        case 0x27:
        case 0x47: tdata[6]=data[2];
            // FALLTHRU
        case 0x2b:
        case 0x4b: tdata[5]=data[1];
            // FALLTHRU
        case 0x2f:
        case 0x4f: tdata[4]=data[0];
            // FALLTHRU
        case 0x40: break;
        default: return 0;
    }
    return (can_send_frame(idt, dlen, tdata) > 0);
}

int sendSDOreq(int node, int object, int subindex){
    unsigned char dummy[1];
    return sendSDOdata(node, 0x40, object, subindex, dummy);
}

int recvSDOresp(int node, int t_func, int t_object, int t_subindex, unsigned char data[]){
    int idt = 0x580|(node&0x7f);
    int dlen = 0;
    canid_t idr;
    unsigned char rdata[8] = {0};
    int ntout = (t_object == 0x1010||t_object == 0x1011)? 50 : 15;
    for(int i = 0; i < ntout; ++i){
        can_dsleep(0.01);
        while(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata)) if(idr==(canid_t)idt){
            int r_func, r_object, r_subindex;
            if(dlen < 4){
                fprintf(stderr,"Too short SDO response from Node%d\n",node&0x7f);
                continue;
            }
            r_func = rdata[0];
            r_object = (rdata[2]<<8)|rdata[1];
            r_subindex = rdata[3];
            if(r_func == 0x80){ // got SDO error code
                unsigned long ercode = (rdata[7]<<24)|(rdata[6]<<16)|(rdata[5]<<8)|rdata[4];
                fprintf(stderr,"SDO error %08lx from Node%d (object %04x/%d) \n",ercode,node&0x7f,r_object,r_subindex);
                fprintf(stderr,"(%s)\n",sdo_abort_text(ercode));
                return 0;
            }
            if(r_object!=t_object || r_subindex != t_subindex){
                fprintf(stderr,"Got SDO response with a stranger object (%04x/%d instead of %04x/%d) from Node%d\n",r_object,r_subindex,t_object,t_subindex,node&0x7f);
                continue;
            }
            if((t_func&0xf0) == 0x20 && r_func == 0x60) return 1;
            if(t_func == 0x40 && (r_func&0xf0) == 0x40){
                dlen = 0;
                switch (r_func & 0x7f){
                    default:
                    case 0x43: data[3] = rdata[7]; dlen++;
                        // FALLTHRU
                    case 0x47: data[2] = rdata[6]; dlen++;
                        // FALLTHRU
                    case 0x4b: data[1] = rdata[5]; dlen++;
                        // FALLTHRU
                    case 0x4f: data[0] = rdata[4]; dlen++;
                    break;
                }
                return dlen;
            }
            fprintf(stderr,"Suspicious SDO response from Node%d (func %02x object %04x/%d)\n",node&0x7f,r_func,r_object,r_subindex);
        }
    }
    fprintf(stderr,"Can't get SDO response from Node%d! Timeout?\n",node&0x7f);
    return 0;
}

int doSDOdownload(int node, int object, int subindex, unsigned char data[], int dlen){
    int func = 0x22;
    switch(dlen){
        default:func = 0x22; break;
        case 4: func = 0x23; break;
        case 3: func = 0x27; break;
        case 2: func = 0x2b; break;
        case 1: func = 0x2f; break;
    }
    can_clean_recv(&rxpnt, &rxtime);
    if(!sendSDOdata(node, func, object, subindex, data)) return 0;
    return recvSDOresp(node, func, object, subindex, data);
}

int doSDOupload(int node, int object, int subindex, unsigned char data[]){
    int func = 0x40;
    can_clean_recv(&rxpnt, &rxtime);
    if(!sendSDOdata(node, func, object, subindex, data)) return 0;
    return recvSDOresp(node, func, object, subindex, data);
}

int setLong(int node, int object, int subindex, unsigned long value){
    unsigned char data[4] = {0};
    data[0] = value&0xff;
    data[1] = (value>>8)&0xff;
    data[2] = (value>>16)&0xff;
    data[3] = (value>>24)&0xff;
    return doSDOdownload(node, object, subindex, data, 4);
}

int setShort(int node, int object, int subindex, unsigned short value){
    unsigned char data[4] = {0};
    data[0] = value&0xff;
    data[1] = (value>>8)&0xff;
    return doSDOdownload(node, object, subindex, data, 2);
}

int setByte(int node, int object, int subindex, unsigned char value){
    unsigned char data[4] = {0};
    data[0] = value;
    return doSDOdownload(node, object, subindex, data, 1);
}

int saveObjects(int node){
    unsigned char data[4] = {'s','a','v','e'};
    return doSDOdownload(node, 0x1010, 1, data, 0);
}

static unsigned char sdata[5] = {'\0'};
char *getString(int node, int object, int subindex){
    int dlen = doSDOupload(node, object, subindex, sdata);
    if(dlen == 0) return NULL;
    sdata[4] = '\0';
    return (char*)sdata;
}

int getLong(int node, int object, int subindex, unsigned long *value){
    unsigned char data[4] = {0};
    int dlen = doSDOupload(node, object, subindex, data);
    if(dlen == 0) return 0;
    if(dlen != 4)
        fprintf(stderr,"Warning! Got only %d bytes for Long value from Node%d/%04x/%d\n",dlen,node,object,subindex);
    *value = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
    return 1;
}

int getShort(int node, int object, int subindex, unsigned short *value){
    unsigned char data[4] = {0};
    int dlen = doSDOupload(node, object, subindex, data);
    if(dlen == 0) return 0;
    if(dlen != 2)
        fprintf(stderr,"Warning! Got %d bytes for Short value from Node%d/%04x/%d\n",dlen,node,object,subindex);
    *value = (data[1]<<8)|data[0];
    return 1;
}

int getByte(int node, int object, int subindex, unsigned char *value){
    unsigned char data[4] = {0};
    int dlen = doSDOupload(node, object, subindex, data);
    if(dlen == 0) return 0;
    if(dlen != 1)
        fprintf(stderr,"Warning! Got %d bytes for Byte value from Node%d/%04x/%d\n",dlen,node,object,subindex);
    *value = data[0];
    return 1;
}

int sendSync(){
// send broadcasting SYNC telegram
    unsigned long idt=0x80;
    int dlen=0;
    unsigned char tdata[1] = {0};
    return (can_send_frame(idt, dlen, tdata) > 0);
}

int recvNextPDO(double tout, int *node, unsigned long *value){
// wait up to 'tout' sec. for the one next PDO
// if ok - return 1 for PDO1 or 2 for PDO2; else 0 if timeout
    double te = can_dtime()+tout;
    int dlen=0, pdon = 0;
    canid_t idr;
    unsigned char rdata[8] = {0};
    do{
        while(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata)){
            if(idr&CAN_RTR_FLAG) continue;
            if((idr&0xf80) == 0x180) pdon=1;
            else if((idr&0xf80) == 0x280) pdon=2;
            if(pdon){
                *node = idr&0x7f;
                *value = (rdata[3]<<24)|(rdata[2]<<16)|(rdata[1]<<8)|rdata[0];
                return pdon;
            }
        }
        can_dsleep(0.02);
    } while(can_dtime() < te);
    return 0;
}

int recvPDOs(double tout, int maxpdo, int node[], int pdo_n[], unsigned long value[]){
// wait up to 'tout' sec. for the array of 'maxpdo' PDOs
// if ok - return number of PDO really received; else 0 if timeout
    double te = can_dtime()+tout;
    int npdo=0, dlen=0, pdon = 0;
    canid_t idr;
    unsigned char rdata[8] = {0};
    do{
        while(can_recv_frame(&rxpnt, &rxtime, &idr, &dlen, rdata)){
            if(idr&CAN_RTR_FLAG) continue;
            if((idr&0xf80) == 0x180) pdon=1;
            else if((idr&0xf80) == 0x280) pdon=2;
            else pdon = 0;
            if(pdon){
                node[npdo] = idr&0x7f;
                pdo_n[npdo] = pdon;
                switch(dlen){
                    case 1:
                        value[npdo] = rdata[0];
                    break;
                    case 2:
                        value[npdo] = (rdata[1]<<8)|rdata[0];
                    break;
                    case 3:
                        value[npdo] = (rdata[2]<<16)|(rdata[1]<<8)|rdata[0];
                    break;
                    default:
                        value[npdo] = (rdata[3]<<24)|(rdata[2]<<16)|(rdata[1]<<8)|rdata[0];
                    break;
                }
                npdo++;
                if(npdo>=maxpdo) return npdo;
            }
        }
        can_dsleep(0.02);
    } while(can_dtime() < te && npdo < maxpdo);
    return npdo;
}

int requestPDO(double tout, int node, int pdon, unsigned long *value){
// send request for PDO and wait up to 'tout' sec. while the node respond to
// if ok - return 1; else 0
    unsigned long idt = (((pdon==1)?0x180:0x280)|(node&0x7f)|CAN_RTR_FLAG);
    int dlen=0, rnode;
    unsigned char dummy[1];
    can_clean_recv(&rxpnt, &rxtime);
    if(can_send_frame(idt, dlen, dummy) <= 0) return 0;
    can_dsleep(0.01);
    if(recvNextPDO(tout, &rnode, value) == pdon) return 1;
    return 0;
}

void clean_recv(){
    can_clean_recv(&rxpnt, &rxtime);
}
