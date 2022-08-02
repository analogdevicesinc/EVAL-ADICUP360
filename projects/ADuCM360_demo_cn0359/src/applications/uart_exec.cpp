/***************************************************************************//**
 *   @file   uart_exec.cpp
 *   @brief  Implementation of uart_exec.cpp
 *   @author
 *******************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <hal/devices.h>
#include <hal/drivers/flash.h>
#include <cstdio>
#include <cstring>
#include <applications/message.h>
#include <applications/uart_exec.h>
#include <applications/command/cmd_poll.h>
#include <applications/command/cmd_voltage.h>
#include <applications/command/cmd_frequency.h>
#include <applications/command/cmd_cellconstant.h>
#include <applications/command/cmd_coefficient.h>
#include <applications/command/cmd_setuptime.h>
#include <applications/command/cmd_holdtime.h>

struct command {
	const char *cmd;
	int (*fun)(int argc, char *argv[]);
};

#define SIZE(a) (sizeof(a)/sizeof(a[0]))

struct command command_table[] = {{"poll", cmd_poll},
	{"setvolt", cmd_voltage},
	{"setfreq", cmd_frequency},
	{"setk", cmd_cellconstant},
	{"setcof", cmd_coefficient},
	{"setstm", cmd_setuptime},
	{"sethtm", cmd_holdtime}
};

int rx_line(int argc, char *argv[])
{
	char line[128];

	rewind(stdin);
	fgets(line, 128, stdin);

	char *p;
	p = strtok(line, ", \t\r\n");

	for (int i = 0; i < SIZE(command_table); ++i) {
		if (!strcmp(p, command_table[i].cmd)) {
			app msg;
			msg.fun = command_table[i].fun;
			msg.argc = 1;
			msg.argv = new char*[1];
			msg.argv[0] = new char[strlen(command_table[i].cmd) + 1];
			strcpy(msg.argv[0], command_table[i].cmd);

			for (p = strtok(nullptr, ", \t\r\n"); p != nullptr;
			     p = strtok(nullptr, ", \t\r\n")) {
				char **old_argv;
				old_argv = msg.argv;

				msg.argv = new char*[msg.argc + 1];

				for (int j = 0; j < msg.argc; ++j) {
					msg.argv[j] = old_argv[j];
				}

				msg.argv[msg.argc] = new char[strlen(p) + 1];

				strcpy(msg.argv[msg.argc], p);

				delete[] old_argv;
				++msg.argc;
			}

			post_message(msg);

			break;
		}
	}
}
