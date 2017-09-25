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
#include "Communication.h"
#include "ADuCM360.h"
#include "UrtLib.h"
#include "DioLib.h"
#include "SpiLib.h"
#include "I2cLib.h"
#include "Test_Config.h"



/************************* Functions Definitions ******************************/

/**
   @brief UART initialization

   @param lBaudrate - UART baud rate
   @param iBits - UART data bits

  @return none

**/
void UART_Init(long lBaudrate, int iBits)
{
   /*Configure UART pins */
#if(UART_PINS == UART_PINS_12)
   DioCfg(pADI_GP0, 0x003C);                    /* Configure P0.1/P0.2 for UART */
#elif(UART_PINS == UART_PINS_67)
   DioCfg(pADI_GP0, 0x9000);                    /* Configure P0.6/P0.7 for UART */
#endif

   UrtCfg(pADI_UART, lBaudrate, iBits, 0); /*  Baud rate = lBaudrate,  5-8 bits */
   UrtMod(pADI_UART, COMMCR_DTR, 0);           /*  Modem Bits*/

   UrtIntCfg(pADI_UART, COMIEN_ERBFI | COMIEN_ETBEI); /* Enables UART interrupt source */
   NVIC_EnableIRQ(UART_IRQn);                  /* Enable UART IRQ */
}

/**
  @brief Write one character to UART

  @param data - Character to write

  @return none

**/
void UART_WriteChar(char data)
{

   UrtTx(pADI_UART, data);   /* Send to UART */

}

/**
  @brief SPI Initialization

  @return none

**/
void SPI_Init(void)
{

   if(SPI_CHANNEL == SPI_PMOD) {    /* Check if SPI0 channel is tested */

      DioPul(pADI_GP1, 0x0F);  /* Disable the internal pull ups on P1[7:4] */

      DioCfg(pADI_GP1, 0xAA00);    /* Configure P1[7:4] for SPI0 */

      SpiBaud(pADI_SPI0, 9, SPIDIV_BCRST_DIS);      /* Set the SPI0 clock rate in Master mode to x kHz. */

      SpiCfg(pADI_SPI0, SPICON_MOD_TX1RX1, SPICON_MASEN_EN, SPICON_CON_EN |
             SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_LOW |
             SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN); /* Configure SPI0 channel */

   } else if (SPI_CHANNEL == SPI_ARDUINO) { /* Check if SPI1 channel is tested */

      DioPul(pADI_GP0, 0xF0);  /* Disable the internal pull ups on P0[3:0] */

      DioCfg(pADI_GP0, 0x0055);    /* Configure P0[3:0] for SPI1 */

      SpiBaud(pADI_SPI1, 2, SPIDIV_BCRST_DIS);      /* Set the SPI1 clock rate in Master mode to x kHz. */

      SpiCfg(pADI_SPI1, SPICON_MOD_TX1RX1, SPICON_MASEN_EN, SPICON_CON_EN |
             SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_LOW |
             SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN); /* Configure SPI1 channel */
   }

}

/**
  @brief Write data to SPI

  @param pSPI - SPI channel
  @param ui8Data - Data to write

  @return none

**/
void SPI_Write( ADI_SPI_TypeDef *pSPI, uint8_t ui8Data)
{
   SpiFifoFlush(pSPI, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);     /* Flush Tx and Rx FIFOs */

   SpiTx(pSPI, ui8Data);    /* Send data to SPI */
}

/**
  @brief I2C Initialization

  @return none

**/
void I2C_Init(void)
{
   if (I2C_PINS == I2C_PINS_0_12) {  /* Check if I2C connection pins are P0.1 - P0.2 */
      DioCfg(pADI_GP0, 0x28);      /* Configure P0.1/P0.2 */
      DioPul(pADI_GP0, 0xF9);      /* Disable pull up on P0.1/P0.2 */

   } else if (I2C_PINS == I2C_PINS_2_01) { /* Check if I2C connection pins are P2.0 - P2.1 */

      DioCfg(pADI_GP2, 0x05);      /* Configure P2.0/P2.1 */
      DioPul(pADI_GP2, 0xFC);      /* Disable pull up on P2.0/P2.1 */
   }

   I2cMCfg(I2CMCON_TXDMA_DIS | I2CMCON_RXDMA_DIS, I2CMCON_IENCMP | I2CMCON_IENRX | I2CMCON_IENTX, I2CMCON_MAS_EN); /* Configure I2C */
   I2cBaud(0x4E, 0x4F); /* Set frequency to 100kHz -  standard mode */

   NVIC_EnableIRQ(I2CM_IRQn);    /* Enable I2C IRQ*/

}

/**
  @brief Write data to I2C

  @param ui8Data - Data to write

  @return none

**/
void I2C_Write(uint8_t ui8Data)
{
   I2cFifoFlush(MASTER, ENABLE);  /* Flush MASTER FIFO */

   I2cMWrCfg(0x10);       /* Configure device address register */

   I2cTx(MASTER, ui8Data);    /* Send data to MASTER */

}


