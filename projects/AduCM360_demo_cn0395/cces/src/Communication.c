/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for communication part.
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2017(c) Analog Devices, Inc.
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
#include "Communication.h"
#include "ADuCM360.h"

#include <ADN8810.h>
#include "SHT30.h"
#include "Timer.h"

#include "SpiLib.h"
#include "DioLib.h"
#include "UrtLib.h"
#include "I2cLib.h"
#include "ClkLib.h"

/************************** Variable Definitions ******************************/

unsigned char           uart_rx_buffer[UART_RX_BUFFER_SIZE];
unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int         uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
unsigned int         uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

uint8_t i2c_rx_buf[6];
uint8_t i2c_rx_cnt = 0;

/************************* Functions Definitions ******************************/

/**
   @brief SPI initialization

   @return none

**/
void SPI_Init(void)
{
   DioPul(pADI_GP0, 0xf8);                               /* Disable the internal pull ups on P0 */

   DioCfg(pADI_GP0, 0x0015);                             /* Configure P0[2:0] for SPI1 */

   DioPulPin(CSAD7988_PORT, CSAD7988_PIN_NUMBER, 0);     /* Disable the internal pull up on AD7988 CS pin */
   DioOenPin(CSAD7988_PORT, CSAD7988_PIN_NUMBER, 1);     /* Set CS pin for AD7988 as output */
   DioClr(CSAD7988_PORT, CSAD7988_PIN);

   DioPulPin(CSADN8810_PORT, CSADN8810_PIN_NUMBER, 0);   /* Disable the internal pull up on ADN8810 CS pin */
   DioOenPin(CSADN8810_PORT, CSADN8810_PIN_NUMBER, 1);   /* Set CS pin for ADN8810 as output */
   DioSet(CSADN8810_PORT, CSADN8810_PIN);

   SpiBaud(pADI_SPI1, 9, SPIDIV_BCRST_DIS);              /* Set the SPI1 clock rate in Master mode to x kHz. */

   /* SPI configuration*/
   SpiCfg(pADI_SPI1, SPICON_MOD_TX2RX2, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
          SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_LOW |
          SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN);

}

/**
   @brief Writes a register to the Converter via SPI.

   @param ui8address - DAC register address
   @param ui8Data - value to be written
   @enMode ui8Data - write mode

   @return none

**/
void SPI_Write(uint8_t ui8address, uint8_t ui8data, enWriteData enMode)
{

   if(SPI_WRITE_DAC_REG == enMode){ /* Select ADN8810 */
      /* Set ADN8810 CS low */
      DioClr(CSADN8810_PORT, CSADN8810_PIN);

      /* Flush Tx and Rx FIFOs */
      SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

      /* Send first byte */
      SpiTx(pADI_SPI1, ui8address); // 4 address bits + 4 data bits
      /* Send second data byte */
      SpiTx(pADI_SPI1, ui8data); // 8 data bits

      /* Wait until 2 bytes are received */
      while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_TWOBYTES) != SPI1STA_RXFSTA_TWOBYTES);

      /* Set ADN8810 CS high */
      DioSet(CSADN8810_PORT, CSADN8810_PIN);
   }
}

/**
   @brief Reads a specified register or two registers address in the accelerometer via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint16_t SPI_Read(void)
{

   //timer_delay_50uS(1); // 0.05ms

   uint8_t ui8AdcUpperCodes = 0;        /* Data register read MSB */
   uint8_t ui8AdcLowerCodes = 0;        /* Data register read LSB */
   uint16_t ui16Result = 0;

   /* Start conversion by briefly setting the CNV pin high*/
   DioSet(CSAD7988_PORT, CSAD7988_PIN);
   /* Set AD7988 CS low */
   DioClr(CSAD7988_PORT, CSAD7988_PIN);

   /* Flush Tx and Rx FIFOs */
   SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

   /* Send dummy bytes in order to receive the register value */
   SpiTx(pADI_SPI1, 0x07);
   SpiTx(pADI_SPI1, 0xAA);

   /* Wait until 2 bytes are received */
   while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_TWOBYTES) != SPI1STA_RXFSTA_TWOBYTES);

   ui8AdcUpperCodes = SpiRx(pADI_SPI1);    /* Data register read MSB */
   ui8AdcLowerCodes = SpiRx(pADI_SPI1);    /* Data register read LSB */

   ui16Result = ((uint16_t)ui8AdcUpperCodes << 8) | ui8AdcLowerCodes;


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
   DioCfgPin(pADI_GP0, PIN6, 1);                    /* Configure P0.6 for UART */
   DioCfgPin(pADI_GP0, PIN7, 2);                    /* Configure P0.7 for UART */

   UrtCfg(pADI_UART, lBaudrate, iBits, 0);          /* Baud rate for 9600, 8-bits */
   UrtMod(pADI_UART, COMMCR_DTR, 0);                /* <<<<< Modem Bits */

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI); /* Enables UART interrupt source */
   NVIC_EnableIRQ(UART_IRQn);                       /* Enable UART IRQ */

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
   char buff[256];

   va_list args;
   va_start (args, fmt);

   vsprintf (buff, fmt, args);
   va_end (args);

   UART_WriteString(buff);
}

/**
  @brief I2C Initialization

  @return none

**/
void I2C_Init(void)
{
   DioCfg(pADI_GP2, 0x05);      /* Configure P2.0/P2.1 */
   DioPul(pADI_GP2, 0xFC);      /* Disable pull up on P2.0/P2.1 */

   I2cMCfg(I2CMCON_TXDMA_DIS | I2CMCON_RXDMA_DIS, I2CMCON_IENCMP | I2CMCON_IENRX | I2CSCON_IENTX_DIS, I2CMCON_MAS_EN); /* Configure I2C */
   I2cBaud(0x4E, 0x4F); /* Set frequency to 100kHz -  standard mode */
   //I2cBaud(0x12, 0x13); /* Set frequency to 400kHz */

   NVIC_EnableIRQ(I2CM_IRQn);    /* Enable I2C IRQ*/

}

/**
  @brief Write data to I2C

  @param ui16Data - Data to write

  @return none

**/
void I2C_Write(uint16_t ui16Data)
{
   uint8_t ui8UpperByte = 0;
   uint8_t ui8LowerByte = 0;

   ui8UpperByte = (uint8_t)((ui16Data & 0xFF00) >> 8);
   ui8LowerByte = (uint8_t)(ui16Data  & 0xFF);

   I2cFifoFlush(MASTER, ENABLE);  // Flush MASTER FIFO

   I2cMWrCfg(SHT30_ADDR);     // Configure device address register

   I2cTx(MASTER, ui8UpperByte);    // Send 1 byte of data
   I2cTx(MASTER, ui8LowerByte);    // Send 1 byte of data
}

/**
  @brief Read data from I2C

  @param none

  @return none

**/
void I2C_Read(void)
{
   I2cFifoFlush(MASTER, ENABLE);  // Flush MASTER FIFO

   I2cMRdCfg(SHT30_ADDR, 6, DISABLE); // Read 6 bytes from device
}
