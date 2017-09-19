/**
******************************************************************************
*   @file     main.c
*   @brief    Project main source file
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2017(c) Analog Devices, Inc.
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

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ADuCM360.h>
#include <AD7988.h>
#include <CN0395.h>

#include "Communication.h"
#include "Timer.h"
#include "UrtLib.h"
#include "I2cLib.h"
#include "GptLib.h"

#include "DioLib.h"

#include <ADN8810.h>
// ----------------------------------------------------------------------------
//
// Standalone $(shortChipFamily) empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

extern uint8_t ui8ContinousRsMeasurement;

extern void CN0395_ComputeHeaterRPT(sMeasurementVariables *sMeasVar);

int main(int argc, char* argv[])
{
   static uint8_t count = 0;

   sMeasurementVariables *sMeasVar;
   sMeasVar = (sMeasurementVariables *)malloc(sizeof(*sMeasVar));
   if (!sMeasVar) {
      return -1;
   }

   timer_start(); // start general purpose timer

   timer_start_us(); // Start the System Tick Timer.

   CN0395_Init(); // Initialize SPI, I2C and UART

   AD7988_Init(); // Initialize ADC

   CN0395_PowerOn(sMeasVar);

  // Infinite loop
  while (1)
    {
        if (uart_cmd == UART_TRUE) {  /* condition becomes true when the system receives as Carriage Return(CR) command from the UART */
              if(count == 0) { // Check first <ENTER> is pressed after reset
                 AppPrintf("\tWelcome to CN0395 application!\n");
                 AppPrintf("\r\nDefault at power up: \n");
                 AppPrintf("\r\n* RS operation mode\n");
                 AppPrintf("\r* IH = 8 mA (Constant current)\n");
                 AppPrintf("\r* Ro = %.2f [Ohms] (Sensor resistance in clean air)\n", sMeasVar->fSensorResCleanAir);
                 AppPrintf("\r\n>>Type in <help> to see available commands\n");

                 count++;
              }
              else { // At a second <ENTER> press, do the processing
                 CN0395_CmdProcess(sMeasVar);
              }
              uart_cmd = UART_FALSE;
              CN0395_CmdPrompt();
        }
        if(ui8ContinousRsMeasurement) {

              uint16_t ui16AdcData = 0;
              float    fHeaterVoltage = 0;

              AD7988_SetOperationMode(AD7988_RH_MODE);
              ui16AdcData = CN0395_ReadAdc(sMeasVar);
              fHeaterVoltage = AD7988_DataToVoltage(ui16AdcData);
              timer_sleep(50); // delay 50ms

              sMeasVar->fHeaterVoltage = fHeaterVoltage;
              CN0395_ComputeHeaterRPT(sMeasVar); // Compute RH, PH and TH

              CN0395_MeasureSensorResistance(sMeasVar);
              timer_sleep(998);
              CN0395_DisplayData(sMeasVar);
        }
    }
}

void I2C0_Master_Int_Handler(void)
{
   uint32_t ui32Status;

   ui32Status = I2cSta(MASTER);  // Copy MASTER status register
   // Check for Rx interrupt
   if((ui32Status & I2CMSTA_RXREQ) == I2CMSTA_RXREQ){
         i2c_rx_buf[i2c_rx_cnt] = I2cRx(MASTER); // load the buffer
         i2c_rx_cnt++;
   }
}


void UART_Int_Handler (void)
{

   unsigned short  status;
   char c;

   status = UrtIntSta(pADI_UART);                      /* Check UART status */

   if (status & COMIIR_NINT) {
      return;   /* Check if UART is busy */
   }

   switch (status & COMIIR_STA_MSK) {             /* Check what command to execute */
   case COMIIR_STA_RXBUFFULL:                     /* Check if UART register is available to be read */

      UART_ReadChar(&c);                          /* Read character from UART */

      switch(c){

         case _BS:
         if (uart_rcnt) {
               uart_rcnt--;
               uart_rx_buffer[uart_rcnt] = 0;
               UART_WriteChar(c, UART_WRITE_IN_INT);
               UART_WriteChar(' ', UART_WRITE_IN_INT);
               UART_WriteChar(c, UART_WRITE_IN_INT);
         }
         break;

         case _CR: /* Check if read character is ENTER */
         uart_cmd = UART_TRUE;                    /* Set flag */
         break;

         default:
         uart_rx_buffer[uart_rcnt++] = c;

         if (uart_rcnt == UART_RX_BUFFER_SIZE) {
            uart_rcnt--;
            UART_WriteChar(_BS, UART_WRITE_IN_INT);
         }

         UART_WriteChar(c, UART_WRITE_IN_INT);
      }

      uart_rx_buffer[uart_rcnt] = '\0';
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

// ----------------------------------------------------------------------------
