/*!
 *****************************************************************************
 * @file:    CN0336.c
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

#include <DioLib.h>
#include <UrtLib.h>

#include "CN0336.h"
#include "AD7091R.h"
#include "Communication.h"
#include "Timer.h"

/********************************* Definitions ********************************/

#if(CALC_FORMULA == TWO_POINT_CALIBRATION)
#define GF        ((IMAX - IMIN)/(float)(ADC_MAX - ADC_MIN))     /* Gain factor formula */
#elif(CALC_FORMULA == TRANSFER_FUNCTION)
#define GAIN         143.75                                  /* Gain value from data sheet -> [V/mA] */
#endif


/************************* Global functions *****************************/

/**
   @brief Initialization part

   @return none

**/
void CN0336_Init(void)
{
   UART_Init(B115200, COMLCR_WLS_8BITS);  /* UART initialization */
}

/**
   @brief Write output data to UART

   @param f32current     - input current value
   @param f32voltage     - output voltage value
   @param u16adc         - ADC code

   @return none

**/
void CN0336_WriteData(float f32current, float f32voltage, uint16_t u16adc)
{
   static  uint16_t u16adcOld = 0;
   uint8_t u8start = 0;

   if ((uart_cmd == UART_TRUE) && (u8StartCounter == 0)) {            /* Check if ENTER key was pressed */

      printf("\tCheck CN0336 data:\n ");     /* Send welcome message */
      printf("\n");
      u8StartCounter++;                  /* Increase counter */
      uart_cmd = UART_FALSE;             /* Prepare for reset */
      u8start++;

   }

   if(( u8start != 0) || ( (((u16adcOld - 2) > u16adc) || ((u16adcOld + 2) < u16adc)) && (u8StartCounter != 0)) ) {

      if(u16adc == INVALID_DATA) {                 /* Check if ADC value is invalid */

         printf("ERROR: Invalid values!!!\n");            /* Send ERROR message */
         printf("Check your settings, please...\n");

      } else {

         printf("\n");

         if (f32current == VALUE_TO_SMALL) {   /* Check if current value is under range */

            printf("Input Current = Check settings: I < %d[mA]\n", IMIN);      /* Send under range message */

         } else if (f32current == VALUE_TO_BIG) {    /* Check if current value is over range */

            printf("Input Current = Check settings: I > %d[mA]\n", IMAX);      /* Send over range message */

         } else {

            printf("Input Current = %.2f[mA]\n", f32current);     /* Send valid current value */
         }

         printf("\n");
         printf("Output Voltage = %.2f[V]\n", f32voltage);     /* Send valid voltage value */
         printf("ADC Code = %d (%#05x)\n", u16adc, u16adc);          /* Send valid ADC code */

      }

      printf("\n");
      printf("-------------------------\n");

   }

   u16adcOld = u16adc;

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

float CN0336_InputCurrent(uint16_t u16adc, float f32voltage)
{
   float f32i = 0;

#if(CALC_FORMULA == TWO_POINT_CALIBRATION)         /* Check which method for calculation was selected */

   if(u16adc < ADC_MIN) {                      /* Check valid boundaries for ADC value */

      f32i = VALUE_TO_SMALL;                /* Current is under range */

   } else if (u16adc > ADC_MAX) {

      f32i = VALUE_TO_BIG;                /* Current is over range */

   } else {

      f32i = IMIN + ( GF * (float)(u16adc - ADC_MIN));            /* Calculate input current */
   }

#elif(CALC_FORMULA == TRANSFER_FUNCTION)

   if (f32voltage < V_OFFSET) {

      f32i = VALUE_TO_SMALL;                /* Current is under range */

   } else if (f32voltage > (VREF - V_OFFSET)) {

      f32i = VALUE_TO_BIG;                /* Current is over range */

   } else {

      f32i = IMIN + (((f32voltage - V_OFFSET) * MULT_FACTOR) / GAIN);        /* Calculate RTD resistance */
   }

#endif

   return f32i;       /* Return RTD resistance value  */
}
#pragma GCC diagnostic pop


/**
   @brief Internal interrupt handler for UART

   @return none

**/
void CN0336_Interrupt(void)
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



