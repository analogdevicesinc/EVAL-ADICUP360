/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for CN0357 communication.
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
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "Communication.h"
#include "ADuCM360.h"


#include "SpiLib.h"
#include "DioLib.h"
#include "UrtLib.h"
#include "ClkLib.h"

/************************** Variable Definitions ******************************/

unsigned char           uart_rx_buffer[UART_RX_BUFFER_SIZE];
unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int         uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
unsigned int         uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

uint8_t convFlag;

/************************* Functions Definitions ******************************/

/**
   @brief SPI initialization
   @return none
**/
void SPI_Init(void)
{

   DioPul(pADI_GP0, 0x00);                    /* Disable the internal pull ups on P0 */

   DioCfg(pADI_GP0, 0x0015);                  /* Configure P0[2:0] for SPI1 */

   DioPulPin(CS_PORT, CS_PIN_NUMBER, 0);         /* Disable the internal pull up on AD7798 CS pin */
   DioOenPin(CS_PORT, CS_PIN_NUMBER, 1);         /* Set CS pin for AD7798 as output */
   DioSet(CS_PORT, CS_PIN);

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
void SPI_Write(unsigned char* data, unsigned char bytesNumber)
{

    uint8_t byte = 0;

    uint16_t ui16fifo_status = (bytesNumber << 8);                                /* Set FIFO status correct value */

    if(convFlag == 0)
       DioClr(CS_PORT, CS_PIN);

    /* Flush Tx and Rx FIFOs */
    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for(byte = 0;byte < bytesNumber;byte++)
    {
          SpiTx(pADI_SPI1, data[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    if(convFlag == 0)
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
void SPI_Read(unsigned char* data, unsigned char bytesNumber)
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

    if(convFlag == 0)
       DioClr(CS_PORT, CS_PIN);

    SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

    for(byte = 0;byte <= bytesNumber;byte++)
    {
       SpiTx(pADI_SPI1, writeData[byte]);
    }

    /* Wait until x bytes are received */
    while ((SpiSta(pADI_SPI1) & ui16fifo_status) != ui16fifo_status);

    data[0] = SpiRx(pADI_SPI1);           /* Dummy read, not needed value */

    for(byte = 0;byte < bytesNumber;byte++)
    {
        data[byte] = SpiRx(pADI_SPI1);
    }

    if(convFlag == 0)
       DioSet(CS_PORT, CS_PIN);
}

/**
  @brief UART initialization

  @param lBaudrate - Baud rate value (see UrtLib.c for values)
  @param iBits - Number of UART bits sent (see UrtLib.c for values)

  @return none

**/
void UART_Init(long lBaudrate, int iBits)
{
   /*Configure UART pins */
   DioCfgPin(pADI_GP0, PIN6, 1);                      /* Configure P0.6 for UART */
   DioCfgPin(pADI_GP0, PIN7, 2);                      /* Configure P0.7 for UART */

   UrtCfg(pADI_UART, lBaudrate, iBits, 0);            /* Baud rate for 9600, 8-bits */
   UrtMod(pADI_UART, COMMCR_DTR, 0);                  /* <<<<< Modem Bits */

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI); /* Enables UART interrupt source */
   NVIC_EnableIRQ(UART_IRQn);                         /* Enable UART IRQ */

}

/**
  @brief Writes one character to UART.

  @param data - Character to write.
  @param mode - Write mode

  @return UART_SUCCESS or error code.

**/
int UART_WriteChar(char data, enWriteData mode)
{
   if(mode == UART_WRITE) {
      UrtTx(pADI_UART, data);

      return UART_SUCCESS;

   } else {
      if (uart_tcnt == UART_TX_BUFFER_SIZE) {

         return UART_NO_TX_SPACE;

      } else {

         if (mode == UART_WRITE_NO_INT) {
            NVIC_DisableIRQ(UART_IRQn);   /* Disable UART IRQ */
         }

         if (uart_tbusy) {
            uart_tx_buffer[(uart_tpos + (uart_tcnt++)) % UART_TX_BUFFER_SIZE] = data;

         } else {
            UrtTx(pADI_UART, data);
            uart_tbusy = UART_TRUE;
         }

         if (mode == UART_WRITE_NO_INT) {
            NVIC_EnableIRQ(UART_IRQn);   /* Enable UART IRQ */
         }

         return UART_SUCCESS;
      }
   }
}

/**
  @brief Writes string to UART.

  @param string - string to write.

  @return UART_SUCCESS or error code.

**/
int UART_WriteString(char *string)
{
   int     result = UART_SUCCESS;


   while (*string != '\0') {
      result = UART_WriteChar(*string++, UART_WRITE_NO_INT);

      if (result != UART_SUCCESS) {
         break;
      }
   }

   return result;
}

/**
  @brief Read character from UART.

  @param data - data to read.

    @return none

**/
void UART_ReadChar(char *data)
{
   *data = (char)UrtRx(pADI_UART);
}



/**
  @brief Writes content of pointer to UART.

  @param const char - point to memory location data.

  @return none

**/
void AppPrintf(const char *fmt, ...)
{
   char buff[1000];                      // changed to 1000 from 256
   va_list args;
   va_start (args, fmt);

   vsprintf (buff, fmt, args);
   va_end (args);

   UART_WriteString(buff);
}

