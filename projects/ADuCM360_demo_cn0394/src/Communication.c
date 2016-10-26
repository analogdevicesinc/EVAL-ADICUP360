/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for communication part.
*   @version  V0.1
*   @author   ADI
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
#include <stdint.h>
#include <string.h>

#include "ADuCM360.h"
#include "SpiLib.h"
#include "DioLib.h"
#include "UrtLib.h"

#include "Communication.h"


/********************************* Global data ********************************/

unsigned char      uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int       uart_tpos, uart_tcnt, uart_cmd, uart_tbusy;

unsigned char ucTxBufferEmpty  = 0;       // Used to indicate that the UART Tx buffer is empty
unsigned char ucWaitForUart = 0;          // Used by calibration routines to wait for user input
unsigned char szTemp[64] = "";            // Used to store string before printing to UART
unsigned char nLen = 0;
unsigned char i = 0;


void UART_Init(long lBaudrate, int iBits)
{

   DioCfgPin(pADI_GP0, PIN6, 1);          // P0.6 as UART RXD
   DioCfgPin(pADI_GP0, PIN7, 2);          // P0.7 as UART TXD


   UrtCfg(pADI_UART, lBaudrate, iBits, 0);      /* Configure UART bus */
   UrtMod(pADI_UART, COMMCR_DTR, 0);           /* Modem Bits */

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI); /* Enables UART interrupt source */
   NVIC_EnableIRQ(UART_IRQn);                  /* Enable UART IRQ */
}


/**
  @brief Read character from UART.

  @param data - data that is received.

  @return none

**/
void UART_ReadChar(char *data)
{
   *data = (char)UrtRx(pADI_UART);         /* Read character from UART */
}


void UART_WriteChar(char c)
{
   ucTxBufferEmpty = 0;
   UrtTx(pADI_UART,c);

   while (ucTxBufferEmpty == 0)
   {
   }

}

int _write (int fd, char *ptr, int len)
{
   char *p = ptr;

   int res = UART_SUCCESS;

   (void)fd;
   (void)len;

   while (*p != '\n') {
      UART_WriteChar(*p++);

      if (res != UART_SUCCESS) {
         break;
      }
   }

   if(*p == '\n') {
      UART_WriteChar('\r');
      UART_WriteChar('\n');
   }

   return res;

}

