/**
******************************************************************************
*   @file     main.c
*   @brief    Project main source file for CN0357
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
#include <stdlib.h>
#include <math.h>

#include <ADuCM360.h>

#include "Communication.h"
#include "Timer.h"
#include "AD7790.h"
#include "AD5270.h"
#include "CN0357.h"
#include "UrtLib.h"

/********************************* Definitions ********************************/

/* Sample pragmas to cope with warnings. Please note the related line at
  the end of this function, used to pop the compiler diagnostics status. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/********************************** Variables **********************************/
#define FEEDBACK_RESISTOR   (1.2 /(float)(SENSOR_RANGE * SENSOR_SENSITIVITY *  0.000000001))

/************************************* Main ************************************/
/**
   @brief The main application function

   @return the function contains infinite loop and never returns/

**/
int main(int argc, char *argv[])
{
   /* Main variables */
   float fAdcVoltage = 0;
   float fConcentration = 0;
   float fRDACvalue = 0;
   float fsensitivity = 0;
   uint8_t ui8Status_Reg = 0;
   uint16_t ui16Adcdata = 0;
   uint16_t ui16RDACdata = 0;


   /* Start the System Tick Timer. */
   timer_start();

   /* Initialize CN0357 components */
   CN0357_Init();

   /* Initialize and set RDAC value of the AD5270 */
   AD5270_Init(FEEDBACK_RESISTOR);

   /* Initialize the AD7790 */
   AD7790_Init();

   /* Compute for the RDAC code nearest to the required feedback resistance */
   ui16RDACdata = (FEEDBACK_RESISTOR / 20000.0) * 1024.0;
   /* Compute for the constants used in voltage and PPM conversion computation */
   fRDACvalue = (float)ui16RDACdata * (20000.0 / 1024.0);
   fsensitivity = ((float)SENSOR_SENSITIVITY * (float)pow(10, -9));

   /* Infinite loop */
   while (1) {

      ui8Status_Reg = AD7790_ReadReg(STATUS_READ);                                  /* Read ADC Status Register*/

      if(ui8Status_Reg == 0x08) {                                                   /* Checks if ADC data is available*/
         ui16Adcdata = AD7790_ReadData();
         fAdcVoltage = AD7790_DataToVoltage(ui16Adcdata);                        /* Convert ADC data to voltage */
         fConcentration = CN0357_CalcPPM(fAdcVoltage, fRDACvalue, fsensitivity); /* Convert voltage to Gas concentration*/
         CN0357_DisplayData(ui16Adcdata, fAdcVoltage, fConcentration);           /* Display data thru UART */
      }


   }

   /* Infinite loop, never returns. */
}

void UART_Int_Handler (void)
{

   unsigned short  status;
   char c;

   status = UrtIntSta(pADI_UART);                      /* Check UART status */

   if (status & COMIIR_NINT) {
      return;   /* Check if UART is busy */
   }

   switch (status & COMIIR_STA_MSK) {                  /* Check what command to execute */
   case COMIIR_STA_RXBUFFULL:                     /* Check if UART register is available to be read */

      UART_ReadChar(&c);                          /* Read character from UART */

      if( c == 13) {                              /* Check if read character is ENTER */
         uart_cmd = UART_TRUE;                    /* Set flag */
      }

      break;


   case COMIIR_STA_TXBUFEMPTY:                                    /* Check if UART register is available to be written */

      if (uart_tcnt) {                                             /* Check uart counter */

         uart_tbusy = UART_TRUE;                                    /* UART is busy with writing*/

         uart_tcnt--;                                               /* Decrease  uart counter */

         UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);   /* Write character to UART */

         if (uart_tpos == UART_TX_BUFFER_SIZE) {                    /* Check if TX buffer is full */
            uart_tpos = 0;                                           /* Reset buffer counter  */
         }

      } else {

         uart_tbusy = UART_FALSE;                               /* UART is no longer busy with writing */
      }

      break;

   default:
      ;
   }

}



#pragma GCC diagnostic pop

/* End Of File */
