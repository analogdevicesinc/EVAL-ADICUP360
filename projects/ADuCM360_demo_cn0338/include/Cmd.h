/**
******************************************************************************
*   @file     Cmd.h
*   @brief    Header file for command line interpreter
*   @version  V0.1
*   @author   ADI
*   @date     February 2016
*  @par Revision History:
*  - V0.1, February 2016: initial version.
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
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
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
**/
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
