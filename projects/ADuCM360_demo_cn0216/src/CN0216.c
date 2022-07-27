/*!
 *****************************************************************************
 * @file:    CN0216.c
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

/***************************** Include Files **********************************/

#include <stdio.h>
#include <stdarg.h>

#include <ADuCM360.h>
#include <UrtLib.h>

#include "CN0216.h"
#include "AD7791.h"
#include "Communication.h"
#include "Timer.h"


/****************************** Global Variables ***********************************/
float fZeroScaleCal;
float fFullScaleCal;
float fGramsPerLsb;


/**
   @brief Initialize CN0216 weight scale application

   @return none

**/
void CN0216_Init(void)
{
   /* Initialize UART port */
   UART_Init (B9600, COMLCR_WLS_8BITS);
   timer_sleep(50);

   /* Calibration routine */
   UART_WriteString("\r\nTaking zero scale calibration measurement");
   UART_WriteString("\r\nPress <ENTER> to calibrate.\r\n");

   fZeroScaleCal = CN0216_CalibrationMeasurement();
   UART_WriteString("\r\nZero scale calibration measurement complete.\r\n ");
   UART_WriteString("\r\nPlease place calibration weight on scale, to obtain full scale calibration point");
   UART_WriteString("\r\nPress <ENTER> to calibrate.");
   fFullScaleCal = CN0216_CalibrationMeasurement();
   UART_WriteString("\r\nFull scale calibration measurement complete.\r\n ");

   /* Calculate gram per LSB value */
   fGramsPerLsb = CN0216_GramsPerCode(fFullScaleCal, fZeroScaleCal);
}


/**
   @brief Calculate Calibration Measurements by averaging 100 samples

   @return float - Calibration reading

**/

float CN0216_CalibrationMeasurement(void)
{
   uint8_t cal = 0;
   uint8_t i = 0;
   uint32_t ui32calibrationData = 0;
   float fCalibrationData = 0.0;

   timer_sleep(100);

   while (cal == 0) {
      if (uart_cmd == UART_TRUE) {
         UART_WriteString("\r\n");

         for (i = 0; i < 100; i++) {
            ui32calibrationData += SPI_Read(DATA_READ , SPI_READ_ADC_DATA);
            timer_sleep(50);
         }

         fCalibrationData = ui32calibrationData / 100.0;
         uart_cmd = UART_FALSE;
         cal = 1;
      }
   }

   return fCalibrationData;
}

/**
   @brief Calculates grams per LSB

   @param fCalFullScale     - Full scale calibration value
   @param fCalZeroScale     - Zero scale calibration value

   @return float       - Value (grams/code)

**/

float CN0216_GramsPerCode (float fCalFullScale, float fCalZeroScale)
{
   float fgramsCode;

   fgramsCode = (float)CAL_WEIGHT / (fCalFullScale - fCalZeroScale);  /* Calculate number of grams per LSB */

   return fgramsCode;
}


/**
   @brief Calculates weight in grams using calibration and ADC data

   @param ui32AdcData     - ADC data register value

   @return float       - Weight (in grams)

**/

float CN0216_WeightCalculation (uint32_t ui32AdcData)
{
   float f32Weight;

   f32Weight = ((float)ui32AdcData - fZeroScaleCal) * fGramsPerLsb;         /* Calculate weight */

   return f32Weight;
}

/**
   @brief Internal printf function via UART

   @param fmt - pointer to data to write
   @param ... - data format

   @return none

**/
void CN0216_Printf(const char *fmt, ...)
{
   char buff[256];

   va_list args;
   va_start (args, fmt);

   vsprintf (buff, fmt, args);
   va_end (args);

   UART_WriteString(buff);
}


