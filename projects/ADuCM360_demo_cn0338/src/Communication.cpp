/**
******************************************************************************
*   @file     Communication.cpp
*   @brief    Source file for communication part.
*   @version  V0.1
*   @author   ADI
*   @date     February 2016
*  @par Revision History:
*  - V0.1, November 2016: initial version.
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
*/

/***************************** Include Files **********************************/
#include <stdio.h>
#include <stdint.h>
#include <cmath>

#include "ADuCM360.h"
#include "DioLib.h"
#include "UrtLib.h"
#include "Communication.h"

#ifdef __cplusplus
extern "C"
{
#endif

/********************************* Global data ********************************/

volatile char uart_rx_queue[UART_RX_QUEUE_SIZE];
volatile int uart_rx_head = 0, uart_rx_tail = 0;
volatile char uart_tx_queue[UART_TX_QUEUE_SIZE];
volatile int uart_tx_head = 0, uart_tx_tail = 0;

char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_head = 0, cmd_tail = 0;

/**
  @brief UART initialization

  @param lBaudrate - UART baud rate

  @return none

**/
void UART_Init(long lBaudrate)
{

   DioCfg(pADI_GP0, 0x003C);                    /* Configure P0.1/P0.2 for UART */

   UrtCfg(pADI_UART, lBaudrate, COMLCR_WLS_8BITS, COMLCR_STOP_EN);      /* Configure UART bus */

   UrtMod(pADI_UART, 0, 0);           /* Modem Bits */

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI); /* Enables UART interrupt source */

}


/**
  @brief Writes one character to UART.

  @param data - Character to write.
  @param mode - Write mode

  @return UART_SUCCESS or error code.

**/
void UART_WriteChar(char c, enWriteData mode)
{
   if (mode == UART_WRITE) {

      uart_tx_queue[uart_tx_tail] = c;

      while ((uart_tx_tail == (uart_tx_head - 1)) || ((uart_tx_tail == (UART_TX_QUEUE_SIZE - 1)) && (uart_tx_head == 0)))
         ;

      if (uart_tx_tail == (UART_TX_QUEUE_SIZE - 1)) {
         uart_tx_tail = 0;

      } else {
         ++uart_tx_tail;
      }

      if (COMLSR_TEMT_BBA) {
         char COMTX = uart_tx_queue[uart_tx_head];

         if (uart_tx_head == (UART_TX_QUEUE_SIZE - 1)) {
            uart_tx_head = 0;

         } else {
            ++uart_tx_head;
         }

         UrtTx(pADI_UART, COMTX);
      }

   } else {
      UrtTx(pADI_UART, c);
   }
}

/**
  @brief Read character from UART.

  @return char - data that is received

**/
char UART_ReadChar(void)
{
   char c;

   c = (char)UrtRx(pADI_UART);

   return c;
}



static volatile auto no_error = true;

/**
  @brief Internal printf function with semihosting via UART.

  @param ptr - data to write.

  @return UART_SUCCESS or error code.

**/
int _write(int fd, char *ptr, int len)
{

   if ((fd == 1) && no_error) {
      for (auto a = 0; a < len; ++a) {

         UART_WriteChar(ptr[a], UART_WRITE);
      }
   }

   if (fd == 2) { //stderr
      no_error = false;

      NVIC_DisableIRQ(UART_IRQn);

      for (auto a = 0; a < len; ++a) {
         while (!COMLSR_TEMT_BBA)
            ;

         UART_WriteChar(ptr[a], UART_WRITE_INT);
      }
   }

   return len;
}

int _read(int fd, char *ptr, int len)
{

   auto l = 0;

   if (fd == 0) {
      for (auto a = 0; a < len; ++a) {
         if (cmd_head != cmd_tail) {
            ptr[a] = cmd_buffer[cmd_head];

            if (cmd_head == (CMD_BUFFER_SIZE - 1)) {
               cmd_head = 0;

            } else {
               ++cmd_head;
            }

            ++l;

         } else {
            break;
         }
      }
   }

   return (l == 0) ? -1 : l;
}

void _ttywrch(int ch)
{

   if (no_error) {

      UART_WriteChar(ch, UART_WRITE);
   }
}

#ifdef __cplusplus
} //extern "C"
#endif

