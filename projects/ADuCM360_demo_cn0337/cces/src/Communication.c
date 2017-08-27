/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for communication part.
*   @version  V0.1
*   @author   ADI
*   @date     October 2015
*  @par Revision History:
*  - V0.1, October 2015: initial version.
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

#include "ADuCM360.h"
#include "SpiLib.h"
#include "DioLib.h"
#include "UrtLib.h"

#include "Communication.h"


/********************************* Global data ********************************/


unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int       uart_tpos, uart_tcnt, uart_cmd, uart_tbusy;


/************************* Functions Definitions ******************************/

/**
   @brief SPI initialization

   @return none

**/
void SPI_Init(void)
{

   DioPul(pADI_GP1, 0x0F);  /* Disable the internal pull ups on P1[7:4] */

   DioCfg(pADI_GP1, 0xAA00);    /* Configure P1[7:4] for SPI0 */

   SpiBaud(pADI_SPI0, 1, SPIDIV_BCRST_DIS);      /* Set the SPI0 clock rate in Master mode to 4MHz. */

   SpiCfg(pADI_SPI0, SPICON_MOD_TX2RX2, SPICON_MASEN_EN, SPICON_CON_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_LOW |
          SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN); /* Configure SPI0 channel */

}

/**
   @brief Receive data from SPI bus

   @param u8Mode - operation mode of the ADC converter
   @param u8Bytes - number of bytes to be read

   @return none

**/
uint16_t SPI_Read (uint8_t u8Mode, enReadBytes u8Bytes)
{

   uint16_t ui16valueL;
   uint16_t ui16valueH;
   uint16_t  ui16Result;


   SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);       /* Flush Tx and Rx FIFOs */

   CONVST_PORT->GPCLR = CONVST_PIN;             /* Start conversion -> CONVST pin low */

   if(u8Mode == 1 ) {                           /* Check if converter is in NORMAL mode */

      CONVST_PORT->GPSET = CONVST_PIN;           /* Pull CONVST pin high */
   }


   SpiTx(pADI_SPI0, 0xaa);               /* Send a dummy byte in order to receive the register value */

   if(u8Bytes == ONE_BYTE) {
      while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_ONEBYTE) != SPI0STA_RXFSTA_ONEBYTE);      /* Wait until 2 bytes are received */

      ui16Result = (uint16_t)SpiRx(pADI_SPI0);       /* Read 1 byte from SPI */

   } else {
      SpiTx(pADI_SPI0, 0xaa);       /* Send second dummy byte in order to receive the register value */

      while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_TWOBYTES) != SPI0STA_RXFSTA_TWOBYTES);      /* Wait until 2 bytes are received */

      ui16valueH = SpiRx(pADI_SPI0);        /* Read first byte from SPI*/

      ui16valueL = SpiRx(pADI_SPI0);       /* Read second byte from SPI */

      ui16Result = (uint16_t)(((ui16valueH << 8) | ui16valueL) >> 4);  /* Store received value as 12 bits format -> MSB */

      if(u8Mode == 2 ) {           /* Check if converter is in POWER-DOWN mode */
         CONVST_PORT->GPSET = CONVST_PIN;    /* Pull CONVST pin high */
      }

   }


   return ui16Result;             /* Return received value */
}

/**
  @brief UART initialization

  @param lBaudrate - UART baud rate
  @param iBits - data length (5 - 8 bits)

  @return none

**/
void UART_Init(long lBaudrate, int iBits)
{

   DioCfg(pADI_GP0, 0x003C);                    /* Configure P0.1/P0.2 for UART */

   UrtCfg(pADI_UART, lBaudrate, iBits, 0);      /* Configure UART bus */
   UrtMod(pADI_UART, COMMCR_DTR, 0);           /* Modem Bits */

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI); /* Enables UART interrupt source */
   NVIC_EnableIRQ(UART_IRQn);                  /* Enable UART IRQ */
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
      result = UART_WriteChar(*string++, UART_WRITE_NO_INT);         /* Write character */

      if (result != UART_SUCCESS) {
         break;
      }
   }

   return result;
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



