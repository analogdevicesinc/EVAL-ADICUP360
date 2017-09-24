/*!
 *****************************************************************************
 * @file:    ADXL362.c
 * @brief:   ADXL362 accelerometer control
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

#include "ADXL362.h"
#include "Communication.h"
#include "Timer.h"


/****************************** Global Data ***********************************/

int16_t i16SensorX;
int16_t i16SensorY;
int16_t i16SensorZ;
int16_t i16SensorT;

volatile uint32_t ui32timer_counter = 0;


/*************************** Module static data *******************************/

/* Accelerometer scan counter */
static uint32_t ui32ScanSensorCounter;

/************************* Global scope functions *****************************/

/**
   @brief Initialization the accelerometer sensor

   @return none

**/
void Sensor_Init(void)
{
   DioPulPin(CSACC_PORT, CSACC_PIN_NUMBER, 0);          /* Disable the internal pull up on CSACC pin */
   DioOenPin(CSACC_PORT, CSACC_PIN_NUMBER, 1);          /* Set CSACC pin as output */

   DioPulPin(INT1ACC_PORT, INT1ACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT1ACC pin */
   DioOenPin(INT1ACC_PORT, INT1ACC_PIN_NUMBER, 0);         /* Set INT1ACC pin as input */

   DioPulPin(INT2ACC_PORT, INT2ACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT2ACC pin */
   DioOenPin(INT2ACC_PORT, INT2ACC_PIN_NUMBER, 0);         /* Set INT2ACC pin as input */

   SPI_Write(SOFT_RESET_REG, 0x52, SPI_WRITE_REG);  /* Soft reset accelerometer */

   timer_sleep(100);                         /* Wait at least 0.5 ms */

   /* Set activity threshold */
   SPI_Write(THRESH_ACT_L, ACT_VALUE & 0xFF, SPI_WRITE_REG);
   SPI_Write(THRESH_ACT_H, ACT_VALUE >> 8, SPI_WRITE_REG);

   SPI_Write(TIME_ACT, (ACT_TIMER / 10), SPI_WRITE_REG);          /* Set activity time at 100Hz data rate */

   /* Set inactivity threshold */
   SPI_Write(THRESH_INACT_L, INACT_VALUE & 0xFF, SPI_WRITE_REG);
   SPI_Write(THRESH_INACT_H, INACT_VALUE >> 8, SPI_WRITE_REG);

   /* Set inactivity time at 100Hz data rate */
   SPI_Write(TIME_INACT_L, ((INACT_TIMER * 100) & 0xFF), SPI_WRITE_REG);
   SPI_Write(TIME_INACT_H, ((INACT_TIMER * 100) >> 8), SPI_WRITE_REG);

   SPI_Write(ACT_INACT_CTL, 0x3F, SPI_WRITE_REG);         /* Set Loop mode, referenced mode for activity and inactivity, enable activity and inactivity functionality */

   SPI_Write(INTMAP1, 0x40, SPI_WRITE_REG);                  /* Map the awake status to INT1 pin */
}

/**
   @brief Turns on accelerometer measurement mode.

   @return none

**/
void Sensor_Start(void)
{
   uint8_t ui8temp;

   Sensor_Delay(1, &ui32ScanSensorCounter, SCAN_SENSOR_TIME);       /* Perform a sensor delay */

   ui8temp = (uint8_t)SPI_Read(POWER_CTL_REG, SPI_READ_ONE_REG);       /* Read POWER_CTL register, before modifying it */

   ui8temp = ui8temp | 0x02;                                          /* Set measurement bit in POWER_CTL register */

   SPI_Write(POWER_CTL_REG, ui8temp, SPI_WRITE_REG);                    /* Write the new value to POWER_CTL register */
}


/**
   @brief Puts the accelerometer into standby mode.

   @return none

**/
void Sensor_Stop(void)
{
   uint8_t ui8temp;

   ui8temp = (uint8_t)SPI_Read(POWER_CTL_REG, SPI_READ_ONE_REG);        /*Read POWER_CTL register, before modifying it */

   ui8temp = ui8temp & 0xFC;                                      /* Clear measurement bit in POWER_CTL register */

   SPI_Write(POWER_CTL_REG, ui8temp, SPI_WRITE_REG);                 /* Write the new value to POWER_CTL register */

}

/**
   @brief Reads the accelerometer data.

   @return none

**/
void Sensor_Scan(void)
{
   if (Sensor_Delay(0, &ui32ScanSensorCounter, SCAN_SENSOR_TIME)) {
      Sensor_Delay(1, &ui32ScanSensorCounter, SCAN_SENSOR_TIME);

      i16SensorX = SPI_Read(XDATA_L_REG, SPI_READ_TWO_REG);

      i16SensorY = SPI_Read(YDATA_L_REG, SPI_READ_TWO_REG);

      i16SensorZ = SPI_Read(ZDATA_L_REG, SPI_READ_TWO_REG);

      i16SensorT = SPI_Read(TEMP_L_REG, SPI_READ_TWO_REG);
   }
}

/**
   @brief Insert a delay for sensor transmitted data

   @param ui8StartFlag - start counter
   @param pu32EndTm - end counter
   @param pu32EndTm - delay time

   @return uint8_t - delay status

**/
uint8_t Sensor_Delay(uint8_t ui8StartFlag, uint32_t *pu32EndTm, uint32_t ui32Delay)
{
   uint8_t ui8Status = 0;

   if (ui8StartFlag) {
      *pu32EndTm = ui32timer_counter + ui32Delay;

   } else {
      if (ui32timer_counter > *pu32EndTm) {
         ui8Status = 1;
      }
   }

   return ui8Status;
}
