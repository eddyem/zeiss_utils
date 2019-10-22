/*
 *                                                                                                  geany_encoding=koi8-r
 * socket.h
 *
 * Copyright 2017 Edward V. Emelianov <eddy@sao.ru, edward.emelianoff@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */
#pragma once
#ifndef __SOCKET_H__
#define __SOCKET_H__

#include "stdbool.h"

// timeout for socket closing
#define SOCKET_TIMEOUT  (5.0)
// default port number (strinig)
#define DEFPORT         "4444"

// commands through the socket
#define S_CMD_STOP      "stop"
#define S_CMD_FOCUS     "focus"
#define S_CMD_TARGSPEED "targspeed"
#define S_CMD_GOTO      "goto"

// answers through the socket
#define S_ANS_ERR       "error"
#define S_ANS_OK        "OK"
#define S_ANS_MOVING    "moving"

bool emerg_stop;

void daemonize(const char *port);
void sock_send_data(const char *host, const char *port, const char *data);

#endif // __SOCKET_H__
