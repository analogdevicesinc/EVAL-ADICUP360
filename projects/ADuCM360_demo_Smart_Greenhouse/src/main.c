/*!
 *****************************************************************************
 * @file:    main.c
 * @brief:   Main source file
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

/***************************** Library Include Files **************************/
#include <stdio.h>
#include <stdlib.h>
#include <ADuCM360.h>

/***************************** Source Include Files ***************************/
#include "Timer.h"
#include "Communication.h"
#include "I2cLib.h"
#include "SpiLib.h"
#include "UrtLib.h"
#include "lcd.h"
#include "CN0398.h"
#include "CN0397.h"
#include "CN0370.h"
#include "Command.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[])
{
   start_acquire = 0;
   convFlag = 0;
   control_system = 0;
   display_enable = 0;
   uart_cmd = UART_FALSE;

   timer_start();

   Comms_Init();
   timer_sleep(500);

   while(uart_cmd == UART_FALSE);
   uart_cmd = UART_FALSE;

   CN0397_Init();
   timer_sleep(500);

   CN0398_Setup();
   timer_sleep(500);

   CN0370_Zero();
   timer_sleep(500);

   //CN0397_StartCal();
   //CN0398_calibrate();

   printf("CftL Demo v0\n");
   printf("Requires: CN0370, CN0397, and CN0398\n");
   printf("Type help for a list of commands\n");

   Command_Prompt();

   while (1)
   {
      timer_sleep(200);

      if (uart_cmd == UART_TRUE)
      {
         Command_Process();
         Command_Prompt();
         uart_cmd = UART_FALSE;
      }

      if (start_acquire == 1)
      {
         SPI1_Disable();
         timer_sleep(1);
         SPI1_Enable();
         CN0397_SetAppData();
         SPI1_Disable();
         timer_sleep(1);
         SPI1_Enable();
         CN0398_set_data();
      }

      if (control_system > 0)
      {
         Command_ControlLux();
      }

      if (display_enable == 1)
      {
         CN0397_DisplayData();
         CN0398_display_data();
      }
   }
}

void UART_Int_Handler(void)
{
   unsigned short  status;
   char c;

   status = UrtIntSta(pADI_UART);

   if (status & COMIIR_NINT)
   {
      return;
   }

   switch (status & COMIIR_STA_MSK)
   {
      case COMIIR_STA_RXBUFFULL:
         c = UART_ReadChar();
         switch (c)
         {
            case _CR:
               uart_cmd = UART_TRUE;
               break;

            case _LF:
               uart_cmd = UART_TRUE;
               break;

            default:
               uart_rx_buffer[uart_rcnt++] = c;
               uart_rx_char = c;
               uart_read_ch = 1;
               break;
         }
         uart_rx_buffer[uart_rcnt] = '\0';
         break;

      case COMIIR_STA_TXBUFEMPTY:
         uart_rdy = 1;
         break;

      default:
         break;
   }
}

void SPI1_Int_Handler (void)
{
   unsigned char uiSPI0STA = 0;

   uiSPI0STA = SpiSta(pADI_SPI0);
   if ((uiSPI0STA & SPI0STA_TX) == SPI0STA_TX)
   {
      spi1TxComplete = 1;
   }
   if ((uiSPI0STA & SPI0STA_RX) == SPI0STA_RX)
   {
      spi1RxComplete = 1;
   }
}

void SPI0_Int_Handler (void)
{
   unsigned char uiSPI0STA = 0;

   uiSPI0STA = SpiSta(pADI_SPI0);
   if ((uiSPI0STA & SPI0STA_TX) == SPI0STA_TX)
   {
      spi0TxComplete = 1;
   }
}

void I2C0_Master_Int_Handler(void)
{
   NVIC_DisableIRQ(I2CM_IRQn);
   unsigned int uiStatus;

   uiStatus = I2cSta(MASTER);

   if((uiStatus & I2CMSTA_RXREQ) == I2CMSTA_RXREQ)
   {
      rxI2Cbuf[rxI2C++] = I2cRx(MASTER);
      if (rxI2C > rxI2Csize - 1)
      {
         rxI2C = 0;
         rxI2Ccomplete++;
      }
   }
   if((uiStatus & I2CMSTA_TCOMP_SET) == I2CMSTA_TCOMP_SET) // Communication Complete
   {
      txI2Ccomplete++;
      rxI2Ccomplete++;
   }
   NVIC_EnableIRQ(I2CM_IRQn);
}

#pragma GCC diagnostic pop

