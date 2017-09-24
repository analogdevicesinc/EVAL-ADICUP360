/*!
 *****************************************************************************
 * @file:    CN0370.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

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

/***************************** Library Include Files **************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************** Source Include Files ***************************/
#include "ADuCM360.h"
#include "Communication.h"
#include "Timer.h"
#include "CN0397.h"

/***************************** Class Variables ********************************/
unsigned int set_red = 0;
unsigned int set_ble = 0;
unsigned int set_grn = 0;

/* Control System Variable */
// 0 - No control
// 1 - bang-bang control
// 2 - pid control
uint8_t control_system = 0;

uint16_t code_led[3] = {0, 0, 0};
float desired_lux[3] = {0, 0, 0};

void CN0370_SetRedLux(float temp)
{
   desired_lux[0] = temp;
}

void CN0370_SetGrnLux(float temp)
{
   desired_lux[1] = temp;
}

void CN0370_SetBleLux(float temp)
{
   desired_lux[2] = temp;
}

void CN0370_SetControlMode(uint8_t temp)
{
   switch(temp)
   {
      case 0:
         control_system = 1;
         printf("Proportional Control\n");
         break;
      case 1:
         control_system = 2;
         break;
      default:
         control_system = 0;
         break;
   }
}

void CN0370_SetRED(unsigned int temp)
{
   if (control_system == 0)
   {
      set_red = temp; // convert string to float
      SPI0_Write(set_red, RED_LED);
   }
   else
   {
      printf("Cannot adjust LED while control system is active\n");
   }
}

void CN0370_SetGRN(unsigned int temp)
{
   if (control_system == 0)
   {
      set_grn = temp; // convert string to float
      SPI0_Write(set_grn, GRN_LED);
   }
   else
   {
      printf("Cannot adjust LED while control system is active\n");
   }
}

void CN0370_SetBLE(unsigned int temp)
{
   if (control_system == 0)
   {
      set_ble = temp; // convert string to float
      SPI0_Write(set_ble, BLE_LED);
   }
   else
   {
      printf("Cannot adjust LED while control system is active\n");
   }
}

void CN0370_Zero(void)
{
   SPI0_Write(0, RED_LED);
   SPI0_Write(0, BLE_LED);
   SPI0_Write(0, GRN_LED);
}

