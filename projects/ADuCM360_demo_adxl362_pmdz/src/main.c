/**
******************************************************************************
*   @file     main.c
*   @brief    Project main source file
*   @version  V0.1
*   @author   ADI
*   @date     May 2016
*  @par Revision History:
*  - V0.1, May 2016: initial version.
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
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
#include <stdlib.h>
#include "diag/Trace.h"

#include <ADUCM360.h>
#include <UrtLib.h>

#include "Timer.h"
#include "ADXL362.h"
#include "Communication.h"


/* Sample pragmas to cope with warnings. Please note the related line at
  the end of this function, used to pop the compiler diagnostics status. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/**
  @brief Looks to see if there is anything available in the UART.

  @return none
**/
void UART_Int_Handler (void)
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

      if( c == 13) {                            /* Check if read character is ENTER */
         uart_cmd = UART_TRUE;                        /* Set flag */
      }

      break;


   case COMIIR_STA_TXBUFEMPTY:                      /* Check if UART register is available to be written */

      if (uart_tcnt) {                              /* Check uart counter */

         uart_tbusy = UART_TRUE;                    /* UART is busy with writing*/

         uart_tcnt--;                               /* Decrease  uart counter */

         UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);          /* Write character to UART */

         if (uart_tpos == UART_TX_BUFFER_SIZE) {                         /* Check if TX buffer is full */
            uart_tpos = 0;                      /* Reset buffer counter  */
         }

      } else {

         uart_tbusy = UART_FALSE;                  /* UART is no longer busy with writing */
      }

      break;

   default:
      ;
   }

}
/**************************** Function Definitions ****************************/

/**
   @brief The main application function

   @return the function contains infinite loop and never returns/

**/

#define TEMP_ADC       0     /* Temperature display units: 1 for ADC, 0 for degrees Celsius */
#define ADXL_SENSE     2     /* ADXL362 sensitivity: 2, 4, 8 [g] */

int main(int argc, char *argv[])

{

   float scale;

#if TEMP_ADC == 0
   float f32temp;
#endif

   /* Initialize SPI1 */
   SPI_Init();

   /* Initialize the System Timer and its interrupt, and starts the System Tick Timer. */
   timer_start();

   /* Initialize UART */
   UART_Init(B9600, COMLCR_WLS_8BITS);

   /* Initialize accelerometer */
   Sensor_Init();

   /* Start accelerometer measurement mode */
   Sensor_Start();

#if ADXL_SENSE == 2
   SPI_Write(FILTER_CONTROL, 0x13, SPI_WRITE_REG);
   scale = 1000.0f;
#endif

#if ADXL_SENSE == 4
   SPI_Write(FILTER_CONTROL, 0x53, SPI_WRITE_REG);
   scale = 500.0f;
#endif

#if ADXL_SENSE == 8
   SPI_Write(FILTER_CONTROL, 0x93, SPI_WRITE_REG);
   scale = 250.0f;
#endif
   // Infinite loop
   while (1)
   {
      Sensor_Scan();

      if (uart_cmd == UART_TRUE) {

      UART_Printf("\r\n X data [G]: %.2f [g]", (float)i16SensorX / scale);
      UART_Printf("\r\n Y data [G]: %.2f [g]", (float)i16SensorY / scale);
      UART_Printf("\r\n Z data [G]: %.2f [g]", (float)i16SensorZ / scale);

#if TEMP_ADC == 0
      f32temp = ((float)i16SensorT + ACC_TEMP_BIAS) / (1 / ACC_TEMP_SENSITIVITY);
      UART_Printf("\r\n The temperature of the ADXL362 is: %.2f [C]", f32temp);
#else
      UART_Printf("\r\n The temp data of the ADXL362 is: %#04d", i16SensorT);
#endif

      UART_Printf("\r\n");

      uart_cmd = UART_FALSE;
      }

      timer_sleep(250);
   }
}

#pragma GCC diagnostic pop

/* End Of File */
