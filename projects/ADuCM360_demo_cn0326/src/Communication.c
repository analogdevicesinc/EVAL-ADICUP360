/*!
 *****************************************************************************
 * @file:    Communication.c
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

/***************************** Include Files **********************************/
#include <stdio.h>
#include <stdint.h>

#include "ADuCM360.h"
#include "SpiLib.h"
#include "DioLib.h"
#include "UrtLib.h"
#include "Timer.h"

#include "Communication.h"


/********************************* Global data ********************************/


unsigned char           uart_rx_buffer[UART_RX_BUFFER_SIZE];
unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int         uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
unsigned int         uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

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
   @brief Reads a specified register address in the converter via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint32_t SPI_Read(uint8_t ui8address, uint8_t ui8bytes)
{

   uint32_t ui32AdcCodes = 0;

   uint8_t ui8counter;
   static uint8_t ui8read_rx;

   uint16_t ui16fifo_status = ((ui8bytes + 1) << 8);             /* Set FIFO status correct value */

   DioClr(CS_PORT, CS_PIN);

   if(ui8address != READ_DATA_REG) {                             /* Check if read command is not for DATA register */

      SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      SpiTx(pADI_SPI0, ui8address);                              /* Write read command to COMM register */

      for(ui8counter = 1; ui8counter <= ui8bytes; ui8counter++) {

         SpiTx(pADI_SPI0, 0xAA);                               /* Write dummy bytes */
      }

      while ((SpiSta(pADI_SPI0) & ui16fifo_status) != ui16fifo_status);

      ui8read_rx = SpiRx(pADI_SPI0);           /* Dummy read, not needed value */

   }

   for(ui8counter = 1; ui8counter <= ui8bytes; ui8counter++) {

      ui8read_rx = SpiRx(pADI_SPI0);                         /* Read the correct 1 byte value */

      ui32AdcCodes = (uint32_t)((ui32AdcCodes << 8) | ui8read_rx);              /* Move read value into 4 bytes value */
   }

   DioSet(CS_PORT, CS_PIN);

   return ui32AdcCodes;
}

/**
   @brief Writes a register to the Converter via SPI.

   @param ui8address - ACC register address
   @param ui8Data - value to be written
   @enMode ui8Data - write mode

   @return none

**/
void SPI_Write(uint8_t ui8address, uint32_t ui32data, uint8_t ui8bytes)
{

   uint8_t ui8counter, ui8write[ui8bytes];
   uint16_t ui16fifo_status = ((ui8bytes + 1) << 8);                                /* Set FIFO status correct value */

   DioClr(CS_PORT, CS_PIN);

   if(ui8bytes != 4) {

      for(ui8counter = 1; ui8counter <= ui8bytes; ui8counter++) {

         ui8write[ui8counter - 1] = (ui32data >> ((ui8bytes - ui8counter) * 8));         /* Separate data into 8 bits values */
      }
   }

   SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);


   if(ui8bytes != 4) {                                                 /* Check if want to write ADC register, not reset the ADC */

      SpiTx(pADI_SPI0, ui8address);                                 /* Send write command to COMM register */

   } else {

      ui16fifo_status = ui8bytes << 8;                           /* Update FIFO status value if ADC reset is required */
   }

   for(ui8counter = 1; ui8counter <= ui8bytes; ui8counter++) {

      if(ui8bytes != 4) {                                     /* Check if want to write ADC register, not reset the ADC */

         SpiTx(pADI_SPI0, ui8write[ui8counter - 1]);          /* Write data */

      } else {

         SpiTx(pADI_SPI0, 0xFF);                           /* Write 4 bytes = 0xFF */
      }

   }

   while ((SpiSta(pADI_SPI0) & ui16fifo_status) != ui16fifo_status);

   DioSet(CS_PORT, CS_PIN);
}

/**
  @brief UART initialization

  @param lBaudrate - UART baud rate
  @param iBits - data length (5 - 8 bits)

  @return none

**/
void UART_Init(long lBaudrate, int iBits)
{

   DioCfg(pADI_GP0, 0x9000);                    /* Configure P0.6/P0.7 for UART */

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
  @brief Read character from UART.

  @param data - data that is received.

  @return none

**/
void UART_ReadChar(char *data)
{
   *data = (char)UrtRx(pADI_UART);         /* Read character from UART */
}

/**
  @brief Internal printf function with semihosting via UART.

  @param ptr - data to write.

  @return UART_SUCCESS or error code.

**/
int _write (int fd, char *ptr, int len)
{
   char *p = ptr;

   int res = UART_SUCCESS;

   (void)fd;
   (void)len;

   while (*p != '\n') {
      res = UART_WriteChar(*p++, UART_WRITE_NO_INT);

      if (res != UART_SUCCESS) {
         break;
      }

      if(*p == '\t') {
         break;
      }
   }

   if(*p == '\n') {
      UART_WriteChar('\r', 1);
      UART_WriteChar('\n', 1);
   }

   return res;

}

