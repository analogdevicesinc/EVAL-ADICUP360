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
#include <stdarg.h>
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

/************************* Functions Definitions ******************************/

/**
   @brief SPI initialization

   @return none

**/
void SPI_Init(void)
{

   DioPul(pADI_GP0, 0x00);                    /* Disable the internal pull ups on P0 */

   DioCfg(pADI_GP0, 0x0055);                  /* Configure P0[3:0] for SPI1 */

   DioPulPin(CSAD7790_PORT, PIN5, 0);         /* Disable the internal pull up on AD7790 CS pin */
   DioOenPin(CSAD7790_PORT, PIN5, 1);         /* Set CS pin for AD7790 as output */
   DioSet(CSAD7790_PORT, CSAD7790_PIN);

   DioPulPin(CSAD5270_PORT, PIN4, 0);         /* Disable the internal pull up on AD5270 CS pin */
   DioOenPin(CSAD5270_PORT, PIN4, 1);         /* Set CS pin for AD5270 as output */
   DioSet(CSAD5270_PORT, CSAD5270_PIN);

   SpiBaud(pADI_SPI1, 9, SPIDIV_BCRST_DIS);   /* Set the SPI1 clock rate in Master mode to x kHz. */

   /* SPI configuration*/
   SpiCfg(pADI_SPI1, SPICON_MOD_TX1RX1, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
          SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);

}


/**
   @brief Writes a register to the Converter via SPI.

   @param ui8address - ACC register address
   @param ui8Data - value to be written
   @enMode ui8Data - write mode

   @return none

**/
void SPI_Write(uint8_t ui8address, uint8_t ui8data, enWriteData enMode)
{
   if(enMode == SPI_WRITE_ADC_REG) { /* Select AD7791 */
      /* Set AD7790 CS low */
      DioClr(CSAD7790_PORT, CSAD7790_PIN);

      /* Flush Tx and Rx FIFOs */
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      /* Send register address */
      SpiTx(pADI_SPI1, ui8address);
      /* Send value to be written */
      SpiTx(pADI_SPI1, ui8data);

      /* Wait until 2 bytes are received */
      while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_TWOBYTES) != SPI1STA_RXFSTA_TWOBYTES);

      /* Set AD7790 CS high */
      DioSet(CSAD7790_PORT, CSAD7790_PIN);

   } else if(enMode == SPI_WRITE_ADC_RESET) {
      /* Set AD7790 CS low */
      DioClr(CSAD7790_PORT, CSAD7790_PIN);

      /* Flush Tx and Rx FIFOs */
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      /* Send all ones to reset part */
      SpiTx(pADI_SPI1, 0xFF);
      SpiTx(pADI_SPI1, 0xFF);
      SpiTx(pADI_SPI1, 0xFF);
      SpiTx(pADI_SPI1, 0xFF);

      /* Wait until 4 bytes are received */
      while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_FOURBYTES) != SPI1STA_RXFSTA_FOURBYTES);

      /* Set AD7790 CS high */
      DioSet(CSAD7790_PORT, CSAD7790_PIN);

   } else if(enMode == SPI_WRITE_POT_REG) /* Select AD5270 */

   {
      /* Set AD5270 CS low */
      DioClr(CSAD5270_PORT, CSAD5270_PIN);

      /* Flush Tx and Rx FIFOs */
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      /* Send register address */
      SpiTx(pADI_SPI1, ui8address);
      /* Send value to be written */
      SpiTx(pADI_SPI1, ui8data);

      /* Wait until 2 bytes are received */
      while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_TWOBYTES) != SPI1STA_RXFSTA_TWOBYTES);

      /* Set AD5270 CS high */
      DioSet(CSAD5270_PORT, CSAD5270_PIN);
   }
}

/**
   @brief Reads a specified register or two registers address in the accelerometer via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint16_t SPI_Read(uint8_t ui8address, enRegsNum enRegs)
{

   uint8_t ui8AdcUpperCodes = 0;        /* Data register read MSB */
   uint8_t ui8AdcLowerCodes = 0;        /* Data register read LSB */
   uint16_t ui16Result = 0;


   if (enRegs == SPI_READ_ADC_REG) {

      /* Set AD7790 CS low */
      DioClr(CSAD7790_PORT, CSAD7790_PIN);

      /* Flush Tx and Rx FIFOs */
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      /* Send read command */
      SpiTx(pADI_SPI1, ui8address);

      /* Send dummy byte in order to receive the register value */
      SpiTx(pADI_SPI1, 0xAA);

      /* Wait until 2 bytes are received */
      while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_TWOBYTES) != SPI1STA_RXFSTA_TWOBYTES);

      ui8AdcLowerCodes = SpiRx(pADI_SPI1);         /* First junk byte, throw away */
      ui8AdcLowerCodes = SpiRx(pADI_SPI1);    /* Data register read MSB */

      ui16Result = (uint16_t)ui8AdcLowerCodes;

      /* Set AD7790 CS high */
      DioSet(CSAD7790_PORT, CSAD7790_PIN);
   }

   if (enRegs == SPI_READ_ADC_DATA) {
      /* Set AD7790 CS low */
      DioClr(CSAD7790_PORT, CSAD7790_PIN);

      /* Flush Tx and Rx FIFOs */
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      /* Send read command */
      SpiTx(pADI_SPI1, ui8address);

      /* Send 2 dummy byte in order to receive the register value */
      SpiTx(pADI_SPI1, 0xAA);
      SpiTx(pADI_SPI1, 0xAA);

      /* Wait until 3 bytes are received */
      while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_THREEBYTES) != SPI1STA_RXFSTA_THREEBYTES);

      ui8AdcUpperCodes = SpiRx(pADI_SPI1);             /* First junk byte */

      ui8AdcUpperCodes = SpiRx(pADI_SPI1);    /* Data register read MSB */
      ui8AdcLowerCodes = SpiRx(pADI_SPI1);    /* Data register read LSB */
      ui16Result = ((uint16_t)ui8AdcUpperCodes << 8) | ui8AdcLowerCodes;

      /*  Set AD7790 CS high */
      DioSet(CSAD7790_PORT, CSAD7790_PIN);
   }

   if (enRegs == SPI_READ_DIGIPOT_REG) {
      /* Set AD5270 CS low */
      DioClr(CSAD5270_PORT, CSAD5270_PIN);

      /* Flush Tx and Rx FIFOs */
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      /* Send 2 dummy byte in order to receive the register value */
      SpiTx(pADI_SPI1, 0x00);
      SpiTx(pADI_SPI1, 0x00);

      /* Wait until 2 bytes are received */
      while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_TWOBYTES) != SPI1STA_RXFSTA_TWOBYTES);

      ui8AdcUpperCodes = SpiRx(pADI_SPI1);    /* Data register read MSB */
      ui8AdcLowerCodes = SpiRx(pADI_SPI1);    /* Data register read LSB */
      ui16Result = ((uint16_t)ui8AdcUpperCodes << 8) | ui8AdcLowerCodes;
      ui16Result &= 0x3FF;    /* Get only the 10 LSB's of the data */

      /*  Set AD5270 CS high */
      DioSet(CSAD5270_PORT, CSAD5270_PIN);
   }


   return ui16Result;
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

