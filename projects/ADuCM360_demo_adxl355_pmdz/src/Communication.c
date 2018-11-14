/*!
 *****************************************************************************
 * @file:    Communications.c
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

#include "ADuCM360.h"
#include "SpiLib.h"
#include "DioLib.h"
#include "UrtLib.h"

#include "Communication.h"

/************************** Variable Definitions ******************************/

unsigned char           uart_rx_buffer[UART_RX_BUFFER_SIZE];
unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int      uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
unsigned int      uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;


/************************* Functions Definitions ******************************/

/**
   @brief SPI initialization

   @return none

**/
void SPI_Init(void)
{
      DioPul(pADI_GP1, 0x00);  // Disable the internal pull ups on P1[3:0]

      DioCfg(pADI_GP1, 0xAA00);    // Configure P1[7:3] for SPI0

      SpiBaud(pADI_SPI0, 9, SPIDIV_BCRST_DIS);      // Set the SPI0 clock rate in Master mode to x kHz.

      SpiCfg(pADI_SPI0, SPICON_MOD_TX1RX1, SPICON_MASEN_EN, SPICON_CON_EN |
             SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_LOW |
             SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN); // Configure SPI0 channel
}


/**
   @brief Writes a data, a command or a register to the LCD or to ACC via SPI.

   @param ui8address - ACC register address
   @param ui8Data - value to be written in 1 register write
   @param ui8Data2 - 2nd value to be written in 2 register write
   @enMode enWriteData - write mode

   @return none

**/
void SPI_Write(uint8_t ui8address, uint8_t ui8Data, uint8_t ui8Data2, enWriteData enMode)
{
   uint8_t ui8writeAddress;
   ui8writeAddress = ((ui8address <<1)|ADXL355_WRITE);

   if(enMode == SPI_WRITE_ONE_REG) {
      DioClr(CSACC_PORT, CSACC_PIN);         /* Select accelerometer */

      SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);      /* Flush Tx and Rx FIFOs */

      SpiTx(pADI_SPI0, ui8writeAddress);     /* Send register address */

      SpiTx(pADI_SPI0, ui8Data);             /* Send value to be written */

      while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_TWOBYTES) != SPI0STA_RXFSTA_TWOBYTES);        /* Wait until 3 bytes are received */

      DioSet(CSACC_PORT, CSACC_PIN);         /* Deselect accelerometer */
   }
   if(enMode == SPI_WRITE_TWO_REG) {
      DioClr(CSACC_PORT, CSACC_PIN);         /* Select accelerometer */

      SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);      /* Flush Tx and Rx FIFOs */

      SpiTx(pADI_SPI0, ui8writeAddress);     /* Send register address */

      SpiTx(pADI_SPI0, ui8Data);             /* Send 1st value to be written */

      SpiTx(pADI_SPI0, ui8Data2);             /* Send 2nd value to be written */

      while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_THREEBYTES) != SPI0STA_RXFSTA_THREEBYTES);        /* Wait until 3 bytes are received */

      DioSet(CSACC_PORT, CSACC_PIN);         /* Deselect accelerometer */
   }
}

/**
   @brief Reads a specified register or two registers address in the accelerometer via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint32_t SPI_Read(uint8_t ui8address, enRegsNum enRegs)
{

   uint32_t ui32Result = 0;

   uint32_t ui32valueL = 0;
   uint32_t ui32valueM __attribute__((unused)) = 0;
   uint32_t ui32valueH = 0;

   uint8_t ui8writeAddress;
   ui8writeAddress = ((ui8address <<1)|ADXL355_READ);

   DioClr(CSACC_PORT, CSACC_PIN);      /* Select accelerometer */

   SpiFifoFlush(pADI_SPI0, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);       /* Flush Tx and Rx FIFOs */

   SpiTx(pADI_SPI0, ui8writeAddress);       /* Send register address */

   SpiTx(pADI_SPI0, 0xAA);               /* Send a dummy byte in order to receive the register value */

   if (enRegs == SPI_READ_ONE_REG) {

      while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_TWOBYTES) != SPI0STA_RXFSTA_TWOBYTES);    /* Wait until 2 bytes are received */

      ui32valueL = SpiRx(pADI_SPI0);            /* One dummy read */

      ui32Result = SpiRx(pADI_SPI0);            /* Set read result*/

   }
   if (enRegs == SPI_READ_TWO_REG) {          /* Only used for Temp & X,Y,Z offset and threshold registers*/

      SpiTx(pADI_SPI0, 0xAA);               /* Send a dummy byte in order to receive the register value */

      while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_THREEBYTES) != SPI0STA_RXFSTA_THREEBYTES);    /* Wait until 3 bytes are received */

      ui32valueL = SpiRx(pADI_SPI0);             /* One dummy read */


      ui32valueH = SpiRx(pADI_SPI0);             /* Read the register value */
      ui32valueL = SpiRx(pADI_SPI0);

      ui32Result = ((ui32valueH << 8) | ui32valueL); /* Set read result*/

   }
   if (enRegs == SPI_READ_THREE_REG) {          /* Only used for X,Y,Z axis data registers*/

      SpiTx(pADI_SPI0, 0xAA);               /* Send a dummy byte in order to receive the register value */
      SpiTx(pADI_SPI0, 0xAA);               /* Send a dummy byte in order to receive the register value */

      while ((SpiSta(pADI_SPI0) & SPI0STA_RXFSTA_FOURBYTES) != SPI0STA_RXFSTA_FOURBYTES);    /* Wait until 4 bytes are received */

      ui32valueL = SpiRx(pADI_SPI0);             /* One dummy read */


      ui32valueH = SpiRx(pADI_SPI0);             /* Read the register value */
      ui32valueM = SpiRx(pADI_SPI0);
      ui32valueL = SpiRx(pADI_SPI0);

      ui32Result = ((ui32valueH << 16) | (ui32valueM << 8) | ui32valueL); /* Set read result*/

   }

   DioSet(CSACC_PORT, CSACC_PIN);          /* Deselect accelerometer */

   return ui32Result;
}

/**
  @brief UART initialization

  @param lBaudrate - Baud rate value
  @param iBits - Number of UART bits sent

  @return none

**/
void UART_Init(long lBaudrate, int iBits)
{
   /*Configure UART pins */
   DioCfgPin(pADI_GP0, PIN6, 1);    /* Configure P0.6 for UART */
   DioCfgPin(pADI_GP0, PIN7, 2);    /* Configure P0.7 for UART */

   UrtCfg(pADI_UART, lBaudrate, iBits, 0); /* Baud rate and x-bits configuration */
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
   @brief Internal printf function via UART

   @param fmt - pointer to data to write
   @param ... - data format

   @return none

**/
void UART_Printf(const char *fmt, ...)
{
   char buff[256];

   va_list args;
   va_start (args, fmt);

   vsprintf (buff, fmt, args);
   va_end (args);

   UART_WriteString(buff);
}
