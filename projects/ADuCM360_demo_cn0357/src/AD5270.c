/*!
 *****************************************************************************
 * @file:    AD5270.c
 * @brief:   AD5270 rheostat
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

/***************************** Include Files **********************************/
#include <stdio.h>
#include <ADuCM360.h>
#include "AD5270.h"
#include "Communication.h"
#include "Timer.h"

/************************* Functions Definitions ******************************/

/**
   @brief Writes a register to the AD5270 via SPI.

   @param ui8command - command to be written
   @param ui16value  - value to be written

   @return none

**/
void AD5270_WriteReg(uint8_t ui8command, uint16_t ui16value)
{
   uint8_t ui8UpperCode = 0;       /* Data register read MSB */
   uint8_t ui8LowerCode = 0;       /* Data register read LSB */
   uint16_t ui16Command = 0;

   /* build 16 bit data to be written - Command + Value */
   ui16Command = ((ui8command & 0x3C) << 8) | (ui16value & 0x3FF);
   ui8UpperCode = (ui16Command >> 8) & 0xFF;
   ui8LowerCode = ui16Command & 0xFF;

   SPI_Write(ui8UpperCode, ui8LowerCode, SPI_WRITE_POT_REG);
}

/**
   @brief Initialization of AD5270

   @return

**/
void AD5270_Init(float fResistorValue)
{
   uint16_t ui16RdacWord = 0;
   ui16RdacWord = AD5270_CalcRDAC(fResistorValue);            /* Compute for the nearest RDAC value from given resistance */
   SPI_Write(WRITE_CTRL_REG, 0x02, SPI_WRITE_POT_REG);        /* write to control register, disable write protect */
   AD5270_WriteReg(WRITE_RDAC, ui16RdacWord);            /* write data to the RDAC register*/

   /* Set AD5270 SDO to Hi-Z */
   AD5270_SetSDOHiZ();
}

/**
   @brief Calculates the value RDAC code from a resistance value

   @param fresistor - resistance value

   @return Equivalent RDAC code computed from the resistance

**/
uint16_t AD5270_CalcRDAC(float fresistor)
{
   uint16_t ui16RdacCode = 0;
   ui16RdacCode = (fresistor / 20000.0) * 1024.0;
   return ui16RdacCode;
}

/**
   @brief Puts the AD5270 SDO line in to Hi-Z mode

   @return none

**/
void AD5270_SetSDOHiZ(void)
{
   SPI_Write(HI_Zupper, HI_Zlower, SPI_WRITE_POT_REG); /* command to prepare SDO for Hi-Z mode */
   SPI_Write(NO_OP, NO_OP, SPI_WRITE_POT_REG);       /* place SDO line in Hi-Z mode */
}


