/*!
 *****************************************************************************
 * @file:    Cmd.h
 * @brief:   Command line interpreter
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2016-2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#pragma once

#include <ctime>


extern time_t start_time;

class CmdTarget
{

public:

   virtual void on_Enter(void);
   virtual void on_Ctrl_c(void);
   virtual void on_NDIR(void);


};

struct CmdTable {
   const char *const p_cmd;
   CmdTarget *const p_target;
   const char *const p_help;
};

#define CMD_MAP_4(_pri, _name, _target, _help) CmdTable _command_##_name __attribute__((section(".cmd_table_"#_pri"_"#_name)))={#_name, _target, _help}
#define CMD_MAP_3(_name, _target, _help) CMD_MAP_4(7, _name, _target, _help)
#define GET_5(arg1, arg2, arg3, arg4, arg5, arg...) arg5
#define CMD_MAP(arg...) GET_5(arg, CMD_MAP_4, CMD_MAP_3)(arg)

#define CMD_OBJ(_name) _command_##_name


class CCmdPrompt: public CmdTarget
{
public:
   void on_Enter(void);
};


extern CmdTarget *pActiveTarget;
extern CCmdPrompt Prompt;
extern CmdTable __cmd_table_start__[], __cmd_table_end__[];


class CCmdHelp: public CmdTarget
{
public:
   void on_Enter(void);
};

extern void Cmd_ReadData(void);


class CCmdStart: public CmdTarget
{
public:
   void on_Enter(void);

};

class CCmdRun: public CmdTarget
{
public:
   void on_Enter(void);
   void on_NDIR(void);
   void on_Ctrl_c(void);
};

class CCmdResetToDefault: public CmdTarget
{
public:
   void on_Enter(void);
};

extern void Cmd_SetMsg(void (*p_fun)(void));
extern void Cmd_ReadMsg(void);
