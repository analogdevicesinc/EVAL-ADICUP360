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

uint8_t convFlag, daisyCh;

void UART_Init(long lBaudrate, int iBits)
{

   //DioCfg(pADI_GP0, 0x9000);                    /* Configure P0.6/P0.7 for UART */
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

void SendString (void)
{
   for ( i = 0 ; i < nLen ; i++ )   // loop to send ADC0 result
   {
      ucTxBufferEmpty = 0;
      UrtTx(pADI_UART,szTemp[i]);
      while (ucTxBufferEmpty == 0)
      {
      }
   }
}
void UART_WriteString(char *data)
{
   sprintf ( (char*)szTemp, "%s", data );
   nLen = strlen((char*)szTemp);
   if (nLen <64)
      SendString();
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

   /*   if(*p == '\t') {
         break;
      }*/
   }

   if(*p == '\n') {
      UART_WriteChar('\r');
      UART_WriteChar('\n');
   }

   return res;

}

/**
   @brief SPI initialization
   @return none
**/
void SPI_Init(void)
{

   DioPul(pADI_GP0, 0xf8);                    /* Disable the internal pull ups on P0 */

   DioCfg(pADI_GP0, 0x0015);                  /* Configure P0[2:0] for SPI1 */

   DioPulPin(AD7798_CS_PORT, AD7798_CS_PIN_NUMBER, 0);         // Disable the internal pull up on AD7798 CS pin
   DioOenPin(AD7798_CS_PORT, AD7798_CS_PIN_NUMBER, 1);         // Set CS pin for AD7798 as output
   DioSet(AD7798_CS_PORT, AD7798_CS_PIN);

   DioPulPin(ADT7310_CS_PORT, ADT7310_CS_PIN_NUMBER, 0);         // Disable the internal pull up on ADT7310 CS pin
   DioOenPin(ADT7310_CS_PORT, ADT7310_CS_PIN_NUMBER, 1);         // Set CS pin for ADT7310 as output
   DioSet(ADT7310_CS_PORT, ADT7310_CS_PIN);

   DioPulPin(AD5270_CS_PORT, AD5270_CS_PIN_NUMBER, 0);         // Disable the internal pull up on ADT7310 CS pin
   DioOenPin(AD5270_CS_PORT, AD5270_CS_PIN_NUMBER, 1);         // Set CS pin for ADT7310 as output
   DioSet(AD5270_CS_PORT, AD5270_CS_PIN);

   SpiBaud(pADI_SPI1, 9, SPIDIV_BCRST_DIS);   /* Set the SPI1 clock rate in Master mode to x kHz. */

   /* SPI configuration*/
   SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
          SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);

}

/***************************************************************************
 * @brief Writes data to SPI.
 *
 * @param data - Write data buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
void SPI_Write(unsigned char* data, unsigned char bytesNumber, enChannels ch)
{

    uint8_t byte = 0;

    uint16_t ui16fifo_status = (bytesNumber << 8);                                /* Set FIFO status correct value */

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 DioClr(AD7798_CS_PORT, AD7798_CS_PIN);
                break;
             case ADT7310:
                 DioClr(ADT7310_CS_PORT, ADT7310_CS_PIN);
                break;
             default:
               {
                  if(daisyCh == 0)
                     DioClr(AD5270_CS_PORT, AD5270_CS_PIN);
               }
                break;
          }
    }

    /* Flush Tx and Rx FIFOs */
    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for(byte = 0;byte < bytesNumber;byte++)
    {
          SpiTx(pADI_SPI1, data[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 DioSet(AD7798_CS_PORT, AD7798_CS_PIN);
                break;
             case ADT7310:
                 DioSet(ADT7310_CS_PORT, ADT7310_CS_PIN);
                break;
             default:
               {
                  if(daisyCh == 0)
                     DioSet(AD5270_CS_PORT, AD5270_CS_PIN);
               }
                break;
          }
    }

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
void SPI_Read(unsigned char* data, unsigned char bytesNumber, enChannels ch)
{

   unsigned char writeData[4]  = {0, 0, 0, 0};
   unsigned char byte          = 0;
   uint16_t ui16fifo_status = ((bytesNumber + 1) << 8);             /* Set FIFO status correct value */

    for(byte = 0;byte <= bytesNumber;byte++)
    {
        if(byte == 0)
           writeData[byte] = data[byte];
        else
           writeData[byte] = 0xAA;    /* dummy value */
    }

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 DioClr(AD7798_CS_PORT, AD7798_CS_PIN);
                break;
             case ADT7310:
                 DioClr(ADT7310_CS_PORT, ADT7310_CS_PIN);
                break;
             default:
                 DioClr(AD5270_CS_PORT, AD5270_CS_PIN);
                break;
          }

    }
    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for(byte = 0;byte <= bytesNumber;byte++)
    {
       SpiTx(pADI_SPI1, writeData[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    data[0] = writeData[0] = SpiRx(pADI_SPI1);           /* Dummy read, not needed value */

    for(byte = 0;byte < bytesNumber;byte++)
    {
        data[byte] = writeData[byte] = SpiRx(pADI_SPI1);
    }

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 DioSet(AD7798_CS_PORT, AD7798_CS_PIN);
                break;
             case ADT7310:
                 DioSet(ADT7310_CS_PORT, ADT7310_CS_PIN);
                break;
             default:
                DioSet(AD5270_CS_PORT, AD5270_CS_PIN);
                break;
          }
    }

}

