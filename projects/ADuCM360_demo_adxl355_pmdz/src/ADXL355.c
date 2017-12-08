/*!
 *****************************************************************************
 * @file:    ADXL355.c
 * @brief:   ADXL355 accelerometer IC
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

/***************************** Include Files **********************************/
#include <stdio.h>

#include <ADuCM360.h>
#include <DioLib.h>

#include "ADXL355.h"
#include "Communication.h"
#include "Timer.h"


/****************************** Global Data ***********************************/

int32_t volatile i32SensorX;
int32_t volatile i32SensorY;
int32_t volatile i32SensorZ;
int32_t volatile i32SensorT;
uint32_t volatile ui32SensorX;
uint32_t volatile ui32SensorY;
uint32_t volatile ui32SensorZ;
uint32_t volatile ui32SensorT;

volatile uint32_t ui32timer_counter = 0;


/************************* Global scope functions *****************************/

/**
   @brief Initialization the accelerometer sensor

   @return none

**/
void ADXL355_Init(void)
{
   DioPulPin(CSACC_PORT, CSACC_PIN_NUMBER, 0);          /* Disable the internal pull up on CSACC pin */
   DioOenPin(CSACC_PORT, CSACC_PIN_NUMBER, 1);          /* Set CSACC pin as output */

   DioPulPin(INT1ACC_PORT, INT1ACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT1ACC pin */
   DioOenPin(INT1ACC_PORT, INT1ACC_PIN_NUMBER, 0);         /* Set INT1ACC pin as input */

   DioPulPin(INT2ACC_PORT, INT2ACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT2ACC pin */
   DioOenPin(INT2ACC_PORT, INT2ACC_PIN_NUMBER, 0);         /* Set INT2ACC pin as input */

   DioPulPin(DATARDYACC_PORT, DATARDYACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT2ACC pin */
   DioOenPin(DATARDYACC_PORT, DATARDYACC_PIN_NUMBER, 0);         /* Set INT2ACC pin as input */

   /* Quick verification test for boards */
//   uint32_t volatile ui32test = SPI_Read(DEVID_AD, SPI_READ_ONE_REG);                  /* Read the ID register */
//   uint32_t volatile ui32test2 = SPI_Read(DEVID_MST, SPI_READ_ONE_REG);                  /* Read the ID register */
//   uint32_t volatile ui32test3 = SPI_Read(PARTID, SPI_READ_ONE_REG);                  /* Read the ID register */
//   uint32_t volatile ui32test4 = SPI_Read(REVID, SPI_READ_ONE_REG);                 /* Read the ID register */

}

/**
   @brief Turns on accelerometer measurement mode.

   @return none

**/
void ADXL355_Start_Sensor(void)
{
   uint8_t ui8temp;

   ui8temp = (uint8_t)SPI_Read(POWER_CTL, SPI_READ_ONE_REG);       /* Read POWER_CTL register, before modifying it */

   ui8temp = ui8temp & 0xFE;                                          /* Set measurement bit in POWER_CTL register */

   SPI_Write(POWER_CTL, ui8temp, 0x00, SPI_WRITE_ONE_REG);                    /* Write the new value to POWER_CTL register */
}


/**
   @brief Puts the accelerometer into standby mode.

   @return none

**/
void ADXL355_Stop_Sensor(void)
{
   uint8_t ui8temp;

   ui8temp = (uint8_t)SPI_Read(POWER_CTL, SPI_READ_ONE_REG);        /*Read POWER_CTL register, before modifying it */

   ui8temp = ui8temp | 0x01;                                      /* Clear measurement bit in POWER_CTL register */

   SPI_Write(POWER_CTL, ui8temp, 0x00, SPI_WRITE_ONE_REG);                 /* Write the new value to POWER_CTL register */

}

/**
   @brief Reads the accelerometer data.

   @return none

**/
void ADXL355_Data_Scan(void)
{

      ui32SensorX = SPI_Read(XDATA3, SPI_READ_THREE_REG);
      ui32SensorY = SPI_Read(YDATA3, SPI_READ_THREE_REG);
      ui32SensorZ = SPI_Read(ZDATA3, SPI_READ_THREE_REG);
      ui32SensorT = SPI_Read(TEMP2, SPI_READ_TWO_REG);
      i32SensorX = ADXL355_Acceleration_Data_Conversion(ui32SensorX);
      i32SensorY = ADXL355_Acceleration_Data_Conversion(ui32SensorY);
      i32SensorZ = ADXL355_Acceleration_Data_Conversion(ui32SensorZ);

}


/**
   @brief Convert the two's complement data in X,Y,Z registers to signed integers

   @param ui32SensorData - raw data from register

   @return int32_t - signed integer data

**/
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData)
{
   int32_t volatile i32Conversion = 0;

   ui32SensorData = (ui32SensorData  >> 4);
   ui32SensorData = (ui32SensorData & 0x000FFFFF);

   if((ui32SensorData & 0x00080000)  == 0x00080000){

         i32Conversion = (ui32SensorData | 0xFFF00000);

   }
   else{
         i32Conversion = ui32SensorData;
   }

   return i32Conversion;
}
