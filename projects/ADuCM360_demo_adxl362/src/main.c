/**
******************************************************************************
*   @file     main.c
*   @brief    Project main source file
*   @version  V0.2
*   @author   ADI
*   @date     October 2015
*  @par Revision History:
*  - V0.1, September 2015: initial version.
*  - V0.2, October 2015: removed ACC defintions and added revision history.
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

#include <DioLib.h>

#include "diag/Trace.h"
#include "Timer.h"

#include "ADXL362.h"
#include "Lcd.h"
#include "Communication.h"


/* Sample pragmas to cope with warnings. Please note the related line at
  the end of this function, used to pop the compiler diagnostics status. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/**************************** Function Definitions ****************************/

/**
   @brief The main application function

   @return the function contains infinite loop and never returns/

**/
int main(int argc, char *argv[])
{
   uint8_t ui8s[22];
   uint8_t ui8xu;
   uint8_t ui8xd;
   uint8_t ui8yu;
   uint8_t ui8yd;
   uint8_t ui8all;
   uint8_t ui8awake;

#if TEMP_ADC == 0
   float f32temp;
#endif

   /* Initialize ports */
   /* Initialize SPI1 */
   SPI_Init();

   /* Initialize the System Timer and its interrupt, and starts the System Tick Timer. */
   timer_start();

   /* Initialize LCD */
   Lcd_Init();

   /* Initialize accelerometer */
   Sensor_Init();

   /* Start accelerometer measurement mode */
   Sensor_Start();

   ui8awake = 0;

   /* Infinite loop */
   while (1) {
      if (DioRd(INTACC_PORT) & INTACC_PIN) {
         if (ui8awake == 0) {
            ui8awake = 1;

            /* Set BLLCD pin - turn on LCD backlight */
            DioSet(BLLCD_PORT, BLLCD_PIN);

            Lcd_DisplayString(0, 60, (int8_t *)"[mG]");
            Lcd_DisplayString(1, 60, (int8_t *)"[mG]");
            Lcd_DisplayString(2, 60, (int8_t *)"[mG]");

#if TEMP_ADC == 1
            Lcd_DisplayString(3, 60, (int8_t *)"[ADC]");
#else
            Lcd_DisplayString(3, 60, (int8_t *)"[C]");
#endif
            ui8xu = 0;
            ui8xd = 0;
            ui8yu = 0;
            ui8yd = 0;
            ui8all = 0;

            Lcd_DisplaySymbol(0, UP_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(1, LEFT_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(1, RIGHT_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(2, DOWN_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(1, CENTER_X, 8, pui8RecInv8x8);
         }

      } else {
         if (ui8awake == 1) {
            ui8awake = 0;

            /* Clear BLLCD pin - turn off LCD backlight */
            DioClr(BLLCD_PORT, BLLCD_PIN);

            /* Clear screen */
            Lcd_FillPages(0, 4, 0x00);
         }
      }

      if (ui8awake == 1) {
         Sensor_Scan();

         sprintf((char *)ui8s, "x = % 5d", i16SensorX);
         Lcd_DisplayString(0, 0, (int8_t *)ui8s);

         sprintf((char *)ui8s, "y = % 5d", i16SensorY);
         Lcd_DisplayString(1, 0, (int8_t *)ui8s);

         sprintf((char *)ui8s, "z = % 5d", i16SensorZ);
         Lcd_DisplayString(2, 0, (int8_t *)ui8s);

#if TEMP_ADC == 1
         sprintf((char *)ui8s, "t = % 5d", i16SensorT);
         Lcd_DisplayString(3, 0, (int8_t *)ui8s);
#else
         f32temp = ((float)i16SensorT - SENS_TEMP_ADD) * SENS_TEMP_MUL;
         sprintf((char *)ui8s, "t = % 4.1f", f32temp);
         Lcd_DisplayString(3, 0, (int8_t *)ui8s);
#endif

         if (i16SensorY > ACC_LIMIT) {
            if (ui8xu == 0) {
               ui8xu = 1;
               Lcd_DisplaySymbol(0, UP_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8xu == 1) {
               ui8xu = 0;
               Lcd_DisplaySymbol(0, UP_X, 8, pui8RecInv8x8);
            }
         }

         if (i16SensorY < -ACC_LIMIT) {
            if (ui8xd == 0) {
               ui8xd = 1;
               Lcd_DisplaySymbol(2, DOWN_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8xd == 1) {
               ui8xd = 0;
               Lcd_DisplaySymbol(2, DOWN_X, 8, pui8RecInv8x8);
            }
         }

         if (i16SensorX > ACC_LIMIT) {
            if (ui8yu == 0) {
               ui8yu = 1;
               Lcd_DisplaySymbol(1, RIGHT_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8yu == 1) {
               ui8yu = 0;
               Lcd_DisplaySymbol(1, RIGHT_X, 8, pui8RecInv8x8);
            }
         }

         if (i16SensorX < -ACC_LIMIT) {
            if (ui8yd == 0) {
               ui8yd = 1;
               Lcd_DisplaySymbol(1, LEFT_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8yd == 1) {
               ui8yd = 0;
               Lcd_DisplaySymbol(1, LEFT_X, 8, pui8RecInv8x8);
            }
         }

         if ((ui8xu == 0) && (ui8xd == 0) && (ui8yu == 0) && (ui8yd == 0)) {
            if (ui8all == 0) {
               ui8all = 1;
               Lcd_DisplaySymbol(1, CENTER_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8all == 1) {
               ui8all = 0;
               Lcd_DisplaySymbol(1, CENTER_X, 8, pui8RecInv8x8);
            }
         }
      }
   }

   /* Infinite loop, never returns. */
}

#pragma GCC diagnostic pop

/* End Of File */
