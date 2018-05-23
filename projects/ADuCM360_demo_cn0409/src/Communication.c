/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for CN0358 communication.
*   @version  V0.1
*   @author   ADI
*   @date     April 2017
*   @par Revision History:
*  - V0.1, April 2017: initial version.
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

#include "ADPD105.h"

#include "I2cLib.h"
#include "DioLib.h"
#include "UrtLib.h"
#include "ClkLib.h"


/************************** Variable Definitions ******************************/

unsigned char           uart_rx_buffer[UART_RX_BUFFER_SIZE];
unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int         uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
unsigned int         uart_echo, uart_cmd, uart_ctrlc, uart_tbusy, uart_received, uart_enter;

uint8_t volatile uiMasterRxIndex = 0;
uint8_t volatile uiMasterTxIndex = 0;
uint8_t volatile uiMasterTxDat[5];
uint8_t volatile uiMasterRxDat[5];

/************************* Functions Definitions ******************************/

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
   @brief Writes a data, a command or a register to the ADT7420 via I2C.

   @param ui8deviceAddress - I2C device address
   @param ui8regAddress - ADT7420 register address
   @param ui8Data - value to be written
   @param ui8Data2 - value to be written
   @param enMode - write mode

   @return none

**/
void I2C_Write(uint8_t ui8deviceAddress, uint8_t ui8regAddress, uint8_t ui8Data, uint8_t ui8Data2, enWriteData enMode)
{
	uiMasterTxIndex = 0;
	ui8deviceAddress = ui8deviceAddress << 1;

	if(enMode == I2C_WRITE_ONE_REG) {

		I2cTx(MASTER, ui8regAddress);                   /* Master sends 8-bit register address */

		I2cTx(MASTER, ui8Data);                         /* Master sends 8-bit data to the register */

		I2cMWrCfg(ui8deviceAddress);                    /* Master sends 7-bit device address w/ write bit (low) */

		while ((pADI_I2C->I2CMSTA  & 0x3) != 0x0)      /* Wait for Tx FIFO to empty */
		{}
	}

	else if(enMode == I2C_WRITE_TWO_REG) {

		NVIC_DisableIRQ(I2CM_IRQn);                   /* Disable I2C Master interrupt source */

		I2cTx(MASTER, ui8regAddress);                   /* Master sends 8-bit register address */

		I2cTx(MASTER, ui8Data);                         /* Master sends 8-bit data to the register */

		I2cMWrCfg(ui8deviceAddress);                    /* Master sends 7-bit device address w/ write bit (low) */

		while ((pADI_I2C->I2CMSTA  & 0x3) != 0x0)     /* Wait for Tx FIFO to empty */
		{}

		I2cTx(MASTER, ui8Data2);                        /* Master sends 8-bit data2 to the register */

		NVIC_EnableIRQ(I2CM_IRQn);                   /* Enable i2C Master interrupt source */
	}

}

/**

   @brief Reads a specified register or two registers address in the ADT7420 via I2C.

   @param ui8deviceAddress - I2C device address
   @param ui8regAddress - ADT7420 register address
   @param enRegs - Read mode

   @return ui16RxResult - reading result

**/
uint16_t I2C_Read(uint8_t ui8deviceAddress, uint8_t ui8regAddress, enRegsNum enRegs)
{
	uiMasterRxIndex = 0;
	uint16_t  ui16RxResult=0;
	ui8deviceAddress = ui8deviceAddress << 1;

	NVIC_DisableIRQ(I2CM_IRQn);                   /* Disable I2C Master interrupt source */

	if (enRegs == I2C_READ_ONE_REG) {

		I2cTx(MASTER, ui8regAddress);

		I2cMWrCfg(ui8deviceAddress);

		while ((pADI_I2C->I2CMSTA  & 0x3) != 0x0)     /* Wait for Tx FIFO to empty */
		{}

		I2cMRdCfg(ui8deviceAddress, 0x01, DISABLE);

		NVIC_EnableIRQ(I2CM_IRQn);                    /* Enable i2C Master interrupt source */

		while (uiMasterRxIndex < 1) {}               /* Wait for 1 Rx interrupt, data read in I2C interrupt handler */

		ui16RxResult = (uint16_t)uiMasterRxDat[0];   /* Set read result*/
	} else if(enRegs == I2C_READ_TWO_REG) {

		I2cTx(MASTER, ui8regAddress);

		I2cMWrCfg(ui8deviceAddress);

		while ((pADI_I2C->I2CMSTA  & 0x3) != 0x0)     /* Wait for Tx FIFO to empty */
		{}

		I2cMRdCfg(ui8deviceAddress, 0x02, DISABLE);

		NVIC_EnableIRQ(I2CM_IRQn);                   /* Enable i2C Master interrupt source */

		while (uiMasterRxIndex < 2) {}               /* Wait for 2 Rx interrupt, data read in I2C interrupt handler */

		ui16RxResult = (((uint16_t)uiMasterRxDat[0] << 8) | (uint16_t)uiMasterRxDat[1]); /* Set read result*/
	}

	return ui16RxResult;
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

