/**
******************************************************************************
*   @file     blink.c
*   @brief    Source file for Blink demo.
*   @version  V0.1
*   @author   ADI
*   @date     August 2015
*
*******************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
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


/***************************** Include Files **********************************/

#include <ADuCM360.h>

#include "blink.h"
#include "DioLib.h"


/******************************************************************************/
/************************* Functions Definitions ******************************/
/******************************************************************************/


/**
   @brief void Blink_Init(void)
         ==========Application required initializations==========
**/
void Blink_Init(void)
{
   /* Set the digital outputs for the LEDs */
   DioOen(pADI_GP0, GP0OEN_OEN4_OUT | GP0OEN_OEN5_OUT);

   /* Initialize output values for LEDs */
   DioWr(pADI_GP0, GP0OUT_OUT4_LOW | GP0OUT_OUT5_HIGH);

}


/**
   @brief void Blink_Process(void)
         ==========Blinking process when not using Timer IRQ==========
**/
void Blink_Process(void)
{
   /* Toggle P0.4 - LED2, P0.5 - LED3 */
   DioTgl(pADI_GP0, GPTGL_TGL4 | GPTGL_TGL5);

}


/***************************************************************************/
