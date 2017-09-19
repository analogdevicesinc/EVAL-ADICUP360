/**
******************************************************************************
*   @file     AD7790.c
*   @brief    Source file for AD7790 ADC control.
*   @version  V0.1
*   @author   ADI
*   @date     December 2015
*   @par Revision History:
*  - V0.1, December 2015: initial version.
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
#include <stdio.h>
#include <math.h>
#include <ADuCM360.h>
#include "AD7790.h"
#include "Communication.h"
#include "Timer.h"

/************************* Functions Definitions ******************************/

/**
   @brief Initialize AD7790 ADC

   @return none

**/
void AD7790_Init(void)
{
   SPI_Write(RESET, RESET, SPI_WRITE_ADC_REG);      /* Reset ADC configuration to default*/
   timer_sleep(1000);
   SPI_Write(MODE_WRITE, 0x00, SPI_WRITE_ADC_REG);  /* Set ADC to unbuffered mode */
   SPI_Write(FILTER_WRITE, 0x07, SPI_WRITE_ADC_REG); /* Set data rate to 9.5Hz */
}

/**
   @brief Read an 8-bit register from the AD7790 via SPI

   @param u8RegAddress - address of the register to be read

   @return 8-bit register value

**/
uint8_t AD7790_ReadReg(uint8_t u8RegAddress)
{
   uint8_t ui8ADC_Reg = 0;

   ui8ADC_Reg = SPI_Read(u8RegAddress, SPI_READ_ADC_REG);

   return ui8ADC_Reg;
}

/**
   @brief Read the 16-bit data register from the AD7790 via SPI

   @return 16-bit conversion data

**/
uint16_t AD7790_ReadData(void)
{
   uint16_t ui16ADC_ReadData = 0;

   ui16ADC_ReadData =  SPI_Read(DATA_READ, SPI_READ_ADC_DATA);

   return ui16ADC_ReadData;
}

/**
   @brief Compute the bipolar ADC input voltage from ADC data

   @param ui16ADCData - 16-bit ADC data value

   @return ADC input voltage

**/
float AD7790_DataToVoltage(uint16_t ui16Adcdata)
{
   float fAdcVoltage = 0;

   fAdcVoltage = (((float)ui16Adcdata / pow(2, 15)) - 1) * 1.2; /* Bipolar voltage computation from ADC code */

   return fAdcVoltage;
}
