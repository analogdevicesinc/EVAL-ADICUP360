/**
******************************************************************************
*   @file     main.cpp
*   @brief    Project main source file
*   @version  V0.1
*   @author   ADI
*   @date     February 2016
*  @par Revision History:
*  - V0.1, February 2016: initial version.
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
#include <cassert>
#include <cstdio>

#include "Timer.h"

#include <ADuCM360.h>
#include <AdcLib.h>
#include <UrtLib.h>

#include "ADC.h"
#include "CN0338.h"
#include "Communication.h"
#include "Cmd.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
   @brief Interrupt handler for UART

   @return none

**/
void UART_Int_Handler(void)
{

   auto COMIIR = UrtIntSta(pADI_UART);

   if ((COMIIR & COMIIR_STA_MSK) == COMIIR_STA_TXBUFEMPTY) {
      if (uart_tx_head != uart_tx_tail) {
         UART_WriteChar(uart_tx_queue[uart_tx_head], UART_WRITE_INT);

         if (uart_tx_head == (UART_TX_QUEUE_SIZE - 1)) {
            uart_tx_head = 0;

         } else {
            ++uart_tx_head;
         }
      }
   }


   if ((COMIIR & COMIIR_STA_MSK) == COMIIR_STA_RXBUFFULL) {
      uart_rx_queue[uart_rx_tail] = UART_ReadChar();

      if (uart_rx_tail == (UART_RX_QUEUE_SIZE - 1)) {
         uart_rx_tail = 0;

      } else {
         ++uart_rx_tail;
      }

      if (uart_rx_tail != uart_rx_head) {
         Cmd_SetMsg(Cmd_ReadData);

      } else {
         if (uart_rx_tail == 0) {
            uart_rx_tail = (UART_RX_QUEUE_SIZE - 1);

         } else {
            --uart_rx_tail;
         }

         assert(false);
      }
   }
}

/**
   @brief Interrupt handler for ADC0

   @return none

**/
void ADC0_Int_Handler(void)
{
   adc0_data = AdcRd(pADI_ADC0);
}

/**
   @brief Interrupt handler for ADC1

   @return none

**/
void ADC1_Int_Handler(void)
{
   adc1_data = AdcRd(pADI_ADC1);

   ADC_IRQ();
}


#ifdef __cplusplus
}
#endif

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/**
   @brief The main application function

   @return The function contains infinite loop and never returns

**/
int
main(int argc, char *argv[])

{
   Timer_start();           /* Start timer */

   CN0338_Init();           /* Initialize application */

   while (1) {
      Cmd_ReadMsg();          /* Check for command */
   }

}

#pragma GCC diagnostic pop


