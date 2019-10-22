/*
 * cmdlnopts.c - the only function that parse cmdln args and returns glob parameters
 *
 * Copyright 2013 Edward V. Emelianoff <eddy@sao.ru>
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
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "cmdlnopts.h"
#include "usefull_macros.h"
#include "socket.h"

/*
 * here are global parameters initialisation
 */
int help;
glob_pars  G;

//            DEFAULTS
// default global parameters
glob_pars const Gdefault = {
    .nodenum = 3,
    .motorID = 12,
    .gotopos = NAN,
    .port = DEFPORT
};

/*
 * Define command line options by filling structure:
 *  name    has_arg flag    val     type        argptr          help
*/
myoption cmdlnopts[] = {
    // set 1 to param despite of its repeating number:
    {"help",    NO_ARGS,    NULL,   'h',    arg_none,   APTR(&help),        "show this help"},
    {"node",    NEED_ARG,   NULL,   'n',    arg_int,    APTR(&G.nodenum),   "encoder node number"},
    {"reset",   NO_ARGS,    NULL,   'r',    arg_none,   APTR(&G.reset),     "reset encoder"},
    {"verbose", NO_ARGS,    NULL,   'v',    arg_int,    APTR(&G.verbose),   "show more info"},
    {"motorid", NEED_ARG,   NULL,   'i',    arg_int,    APTR(&G.motorID),   "motor controller address"},
    //{"bcastid", NEED_ARG,   NULL,   'b',    arg_int,    APTR(&G.motorID),   "motor controller broadcast address"},
    {"gotopos", NEED_ARG,   NULL,   'g',    arg_double, APTR(&G.gotopos),   "target focus position"},
    {"targspeed",NEED_ARG,  NULL,   't',    arg_double, APTR(&G.targspeed), "move motor with constant speed (rev/min)"},
    {"stop",    NO_ARGS,    NULL,   's',    arg_none,   APTR(&G.stop),      "stop motor"},
    {"monitor", NEED_ARG,   NULL,   'm',    arg_double, APTR(&G.monitspd),  "move a little with given speed with monitoring"},
    {"eswstate",NO_ARGS,    NULL,   'e',    arg_none,   APTR(&G.showesw),   "show end-switches state"},
    {"logfile", NEED_ARG,   NULL,   'l',    arg_string, APTR(&G.logname),   "logfile name and path"},
    {"server",  NO_ARGS,    NULL,   'S',    arg_none,   APTR(&G.server),    "work as server"},
    {"port",    NEED_ARG,   NULL,   'p',    arg_string, APTR(&G.port),      "server port number"},
    {"host",    NEED_ARG,   NULL,   'H',    arg_string, APTR(&G.host),      "host to connect (default: localhost)"},
    {"standalone",NO_ARGS,  NULL,   'A',    arg_none,   APTR(&G.standalone),"run as standalone application"},
    end_option
};

/**
 * Parse command line options and return dynamically allocated structure
 *      to global parameters
 * @param argc - copy of argc from main
 * @param argv - copy of argv from main
 * @return allocated structure with global parameters
 */
glob_pars *parse_args(int argc, char **argv){
    void *ptr = memcpy(&G, &Gdefault, sizeof(G)); assert(ptr);
    // format of help: "Usage: progname [args]\n"
    change_helpstring("Usage: %s [args]\n\n\tWhere args are:\n");
    // parse arguments
    parseargs(&argc, &argv, cmdlnopts);
    if(help) showhelp(-1, cmdlnopts);
    return &G;
}

