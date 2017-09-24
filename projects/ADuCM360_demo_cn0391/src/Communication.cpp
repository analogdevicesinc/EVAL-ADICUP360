/*!
 *****************************************************************************
 * @file:    Communication.cpp
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

#include <ADuCM360.h>
#include <DioLib.h>
#include <SpiLib.h>
#include <UrtLib.h>

#include <stdio.h>
#include <string.h>
#include "Communication.h"


unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned char ucTxBufferEmpty  = 0;       // Used to indicate that the UART Tx buffer is empty
unsigned char ucWaitForUart = 0;          // Used by calibration routines to wait for user input
unsigned char szTemp[64] = "";            // Used to store string before printing to UART
unsigned char nLen = 0;
unsigned char i = 0;

uint8_t convFlag = 0, readConv;

SPIClass SPI;
UARTClass UART;

void SPIClass::Init()
{
   DioPulPin(pADI_GP0, PIN0, 0);
   DioCfgPin(pADI_GP0, PIN0, 1);        //MISO
   DioPulPin(pADI_GP0, PIN1, 0);
   DioCfgPin(pADI_GP0, PIN1, 1);        //SCLK
   DioPulPin(pADI_GP0, PIN2, 0);
   DioCfgPin(pADI_GP0, PIN2, 1);        //MOSI

   DioPulPin(CS_PORT, CS_PIN_NUMBER, 0);         /* Disable the internal pull up on AD7798 CS pin */
   DioOenPin(CS_PORT, CS_PIN_NUMBER, 1);         /* Set CS pin for AD7798 as output */
   DioSet(CS_PORT, CS_PIN);

   SpiBaud(pADI_SPI1, 9, SPIDIV_BCRST_DIS);   /* Set the SPI1 clock rate in Master mode to x kHz. */

   /* SPI configuration*/
   SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
          SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);

}

void SPIClass::Write(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber)
{

    uint8_t byte = 0, reset = 0;

    uint16_t ui16fifo_status;

    if(bytesNumber == 8){

       bytesNumber = 4;
       reset = 1;
    }
    ui16fifo_status = (bytesNumber << 8);                                /* Set FIFO status correct value */

    if(slaveDeviceId == 0)
       DioClr(CS_PORT, CS_PIN);

   /* Flush Tx and Rx FIFOs */
   SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

   for(byte = 0;byte < bytesNumber;byte++)
   {
		 SpiTx(pADI_SPI1, data[byte]);
   }

   /* Wait until x bytes are received */
   while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

   if(reset == 1){

		 SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

		 for(byte = 0;byte < bytesNumber;byte++)
		 {
			   SpiTx(pADI_SPI1, data[byte]);
		 }


		 while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);
   }

   if(slaveDeviceId == 0)
	 DioSet(CS_PORT, CS_PIN);

}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
void SPIClass::Read(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber)
{

   unsigned char writeData[4]  = {0, 0, 0, 0};
   unsigned char byte          = 0;
   uint16_t ui16fifo_status;


   ui16fifo_status = ((bytesNumber) << 8);             /* Set FIFO status correct value */

    for(byte = 0;byte < bytesNumber;byte++)
    {
        if(byte == 0)
           writeData[byte] = data[byte];
        else
           writeData[byte] = 0xAA;    /* dummy value */
    }

    if(slaveDeviceId == 0)
       DioClr(CS_PORT, CS_PIN);

    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for(byte = 0;byte < bytesNumber;byte++)
    {
       SpiTx(pADI_SPI1, writeData[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    for(byte = 0;byte < bytesNumber;byte++)
    {
        data[byte] = SpiRx(pADI_SPI1);
    }

    if(slaveDeviceId == 0)
       DioSet(CS_PORT, CS_PIN);
}



void UARTClass::Init()
{
   DioCfgPin(pADI_GP0, PIN6, 1);          // P0.6 as UART RXD
   DioCfgPin(pADI_GP0, PIN7, 2);          // P0.7 as UART TXD


   UrtCfg(pADI_UART, B19200, COMLCR_WLS_8BITS, 0);
   UrtMod(pADI_UART, COMMCR_DTR, 0);

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI);
   NVIC_EnableIRQ(UART_IRQn);

}


void UARTClass::WriteChar(char c)
{
   ucTxBufferEmpty = 0;
   UrtTx(pADI_UART,c);

   while (ucTxBufferEmpty == 0)
   {
   }

}


#ifdef __cplusplus
extern "C"
{
#endif
int _write (int fd, char *ptr, int len)
{
   char *p = ptr;

   int res = UART_SUCCESS;

   (void)fd;
   (void)len;

   while (*p != '\n') {
      UART.WriteChar(*p++);

      if (res != UART_SUCCESS) {
         break;
      }
   }

   if(*p == '\n') {
         UART.WriteChar('\r');
         UART.WriteChar('\n');
   }

   return res;

}

#ifdef __cplusplus
} //extern "C"
#endif




