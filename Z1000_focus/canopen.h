// (c) vsher@sao.ru
#pragma once
#ifndef CANOPEN_H__
#define CANOPEN_H__

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "can_io.h"

#define startNode(Node) sendNMT(Node,1) // put node in "Operational" mode
#define stopNode(Node) sendNMT(Node,2)  // put node in "Stop" mode
#define setPreOper(Node) sendNMT(Node,0x80) // return node in "Pre-Operational" mode

#define NodeStopped 4
#define NodeOperational 5
#define NodePreOperational 0x7f

void clean_recv();
int initNode(int node);
int sendNMT(int node, int icode);
int resetNode2(int oldnode, int newnode);
int resetNode(int node);
int getNodeState(int node);
int initNode(int node);
int sendSDOdata(int node, int func, int object, int subindex, unsigned char data[]);
int sendSDOreq(int node, int object, int subindex);
int recvSDOresp(int node, int t_func, int t_object, int t_subindex, unsigned char data[]);
int doSDOdownload(int node, int object, int subindex, unsigned char data[], int dlen);
int doSDOupload(int node, int object, int subindex, unsigned char data[]);
int setLong(int node, int object, int subindex, unsigned long value);
int setShort(int node, int object, int subindex, unsigned short value);
int setByte(int node, int object, int subindex, unsigned char value);
int saveObjects(int node);
char *getString(int node, int object, int subindex);
int getLong(int node, int object, int subindex, unsigned long *value);
int getShort(int node, int object, int subindex, unsigned short *value);
int getByte(int node, int object, int subindex, unsigned char *value);
int sendSync();
int recvNextPDO(double tout, int *node, unsigned long *value);
int recvPDOs(double tout, int maxpdo, int node[], int pdo_n[], unsigned long value[]);
int requestPDO(double tout, int node, int pdon, unsigned long *value);

#endif // CANOPEN_H__
