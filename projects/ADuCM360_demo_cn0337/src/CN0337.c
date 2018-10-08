/*!
 *****************************************************************************
 * @file:    CN0337.c
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

#include <DioLib.h>
#include <UrtLib.h>

#include "CN0337.h"
#include "AD7091R.h"
#include "Communication.h"
#include "Timer.h"

/********************************* Definitions ********************************/

#if(RTD_FORMULA == TWO_POINT_CALIBRATION)
#define GF         ((RMAX - RMIN)/(ADC_MAX - ADC_MIN))      /* Gain factor formula */
#elif(RTD_FORMULA == TRANSFER_FUNCTION)
#define GF         0.02053                                  /* Gain value from data sheet -> [A] */
#define V_OFFSET   0.1                                      /* Voltage offset of the circuit from data sheet -> [V] */
#endif

/********************************* Global data ********************************/

/* RTD temperature look-up table for 100 ohmi - 212.05 ohmi resistance */
const float C_rtd[] = {0, 2.86808, 5.73874, 8.61185, 11.4874, 14.3654, 17.2459, 20.1288, 23.0142, 25.9021,
                       28.7925, 31.6853, 34.5807, 37.4786, 40.379, 43.2819, 46.1873, 49.0952, 52.0057, 54.9188,
                       57.8344, 60.7525, 63.6732, 66.5965, 69.5224, 72.4508, 75.3819, 78.3155, 81.2517, 84.1906,
                       87.1321, 90.0762, 93.0229, 95.9723, 98.9243, 101.879, 104.836, 107.796, 110.759, 113.724,
                       116.692, 119.663, 122.637, 125.613, 128.592, 131.573, 134.558, 137.545, 140.535, 143.527,
                       146.523, 149.521, 152.522, 155.526, 158.532, 161.542, 164.554, 167.569, 170.586, 173.607,
                       176.631, 179.657, 182.686, 185.718, 188.753, 191.791, 194.832, 197.875, 200.922, 203.971,
                       207.024, 210.079, 213.137, 216.199, 219.263, 222.33, 225.4, 228.473, 231.549, 234.628,
                       237.71, 240.796, 243.884, 246.975, 250.069, 253.167, 256.267, 259.37, 262.477, 265.587,
                       268.699, 271.815, 274.934, 278.056, 281.182, 284.31, 287.442, 290.576, 293.714, 296.855, 300
                      };


/************************* Global functions *****************************/

/**
   @brief Initialization part

   @return none

**/
void CN0337_Init(void)
{
   UART_Init(B9600, COMLCR_WLS_8BITS);  /* UART initialization */
}


/**
   @brief Calculate RTD resistance

   @param u16adc - ADC code

   @return float - resistance value

**/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

float CN0337_CalculateResistance(uint16_t u16adc, float f32voltage)
{
   float f32r = 0;

#if(RTD_FORMULA == TWO_POINT_CALIBRATION)         /* Check which method for calculation was selected */

   if(u16adc < ADC_MIN) {                      /* Check valid boundaries for ADC value */

      f32r = VALUE_TO_SMALL;                /* RTD resistance and temperature are under range */

   } else if (u16adc > ADC_MAX) {

      f32r = VALUE_TO_BIG;                /* RTD resistance and temperature are over range */

   } else {

      f32r = RMIN + (GF * (float)(u16adc - ADC_MIN));             /* Calculate RTD resistance */
   }

#elif(RTD_FORMULA == TRANSFER_FUNCTION)

   if (f32voltage < V_OFFSET) {

      f32r = VALUE_TO_SMALL;                /* RTD resistance and temperature are under range */

   } else if (f32voltage > (VREF - V_OFFSET)) {

      f32r = VALUE_TO_BIG;                /* RTD resistance and temperature are over range */

   } else {

      f32r = RMIN + ((f32voltage - V_OFFSET) / GF);          /* Calculate RTD resistance */
   }

#endif

   return f32r;       /* Return RTD resistance value  */
}

#pragma GCC diagnostic pop

/**
   @brief Calculate RTD temperature

   @param u16adc - RTD resistance

   @return float - temperature value

**/
float CN0337_CalculateTemp(float r)
{
   float t;
   int i = 0;

   if((r == VALUE_TO_SMALL) || (r == VALUE_TO_BIG) ) {

      t = r;                         /* Set temperature value as under or over range */

   } else {

      i = (r - RMIN) / RSEG;                          /* Calculate which coefficient to use from look-up table */

      t = C_rtd[i] + (r - (RMIN + RSEG * i)) * (C_rtd[i + 1] - C_rtd[i]) / RSEG; /* Calculate RTD temperature */
   }

   return t;

}

/**
   @brief Write output data to UART

   @param f32temp     - RTD temperature value
   @param f32r        - RTD resistance value
   @param f32voltage - Voltage value
   @param u16adc   - ADC code

   @return none

**/
void CN0337_WriteData(float f32temp, float f32r, float f32voltage, uint16_t u16adc )
{

   if (uart_cmd == UART_TRUE) {            /* Check if ENTER key was pressed */

      if(u8StartCounter == 0) {                 /* Check if initialization us done */

         UART_WriteString("\r\t Check CN0337 output data:\n\n ");     /* Send welcome message */
         u8StartCounter++;                  /* Increase counter */
      }

      if(u16adc == INVALID_DATA) {                 /* Check if ADC value is invalid */

         CN0337_Printf("\r\nERROR: Invalid values!!!");            /* Send ERROR message */
         CN0337_Printf("\r\nCheck your settings, please...");

      } else {

         if (f32r == VALUE_TO_SMALL) {   /* Check if resistance value is under range => temperature is also under range */

            /* Send under range messages */
            CN0337_Printf("\r\nTemperature\t= Invalid value: t < %d[˚C]", TMIN);
            CN0337_Printf("\r\nResistance\t= Invalid value: R < %d[Ω]", RMIN);

         } else if (f32r == VALUE_TO_BIG) { /* Check if resistance value is over range => temperature is also over range */

            /* Send over range messages */
            CN0337_Printf("\r\nTemperature\t= Invalid value: t > %d[˚C]", TMAX);
            CN0337_Printf("\r\nResistance\t= Invalid value: R > %.2f[Ω]", RMAX);


         } else {                               /* For valid data */

            CN0337_Printf("\r\nTemperature\t= %.4f[˚C]", f32temp);  /* Send valid temperature value */
            CN0337_Printf("\r\nResistance\t= %.4f[Ω]", f32r);       /* Send valid resistance value */

         }

         CN0337_Printf("\r\nVoltage \t= %.4f[V]", f32voltage);     /* Send valid voltage value */
         CN0337_Printf("\r\nADC Code\t= %d (%#05x)", u16adc, u16adc);          /* Send valid ADC code */

      }

      UART_WriteString("\r\n");
      uart_cmd = UART_FALSE;            /* Reset flag */
   }

}

/**
   @brief Internal printf function via UART

   @param fmt - pointer to data to write
   @param ... - data format

   @return none

**/
void CN0337_Printf(const char *fmt, ...)
{
   char buff[256];

   va_list args;
   va_start (args, fmt);

   vsprintf (buff, fmt, args);
   va_end (args);

   UART_WriteString(buff);
}

/**
   @brief Internal interrupt handler for UART

   @return none

**/
void CN0337_Interrupt(void)
{

   unsigned short  status;
   char c;

   status = UrtIntSta(pADI_UART);                      /* Check UART status */

   if (status & COMIIR_NINT) {
      return;   /* Check if UART is busy */
   }

   switch (status & COMIIR_STA_MSK) {                  /* Check what command to execute */
   case COMIIR_STA_RXBUFFULL:                         /* Check if UART register is available to be read */

      UART_ReadChar(&c);                             /* Read character from UART */

      if( c == _CR_KEY) {                            /* Check if read character is ENTER */
         uart_cmd = UART_TRUE;                        /* Set flag */
      }

      break;

   case COMIIR_STA_TXBUFEMPTY:                      /* Check if UART register is available to be written */

      if (uart_tcnt) {                              /* Check uart counter */

         uart_tbusy = UART_TRUE;                    /* UART is busy with writing*/

         uart_tcnt--;                               /* Decrease  uart counter */

         UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);          /* Write character to UART */

         if (uart_tpos == UART_TX_BUFFER_SIZE) {                         /* Check if TX buffer is full */
            uart_tpos = 0;                               /* Reset buffer counter  */
         }

      } else {

         uart_tbusy = UART_FALSE;                  /* UART is no longer busy with writing */
      }

      break;

   default:
      ;
   }

}



