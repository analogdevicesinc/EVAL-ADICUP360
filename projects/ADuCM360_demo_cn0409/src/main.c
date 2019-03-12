/***************************************************************************//**
 *   @file   main.c
 *   @brief  Main function implementation.
 *   @author Mircea Caprioru (mircea.caprioru@analog.com)
********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

// ----------------------------------------------------------------------------

#include <CN0409.h>
#include <stdio.h>
#include <stdlib.h>
#include <ADuCM360.h>

#include "Timer.h"
#include "Communication.h"
#include "ADPD105.h"
#include <I2cLib.h>
#include <UrtLib.h>
#include "Flash.h"

// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/******************************************************************************/
/************************** Variable Definitions ******************************/
/******************************************************************************/
uint16_t volatile ui16I2cStatus = 0;

char s[100] = "hello";
uint8_t _index = 0;

/******************************************************************************/
/************************* Functions Definitions ******************************/
/******************************************************************************/

/**
  @brief UART Interrupt Handler

  @return none

**/
void UART_Int_Handler (void)
{
	unsigned short  status;
	char c;

	/* Check UART status */
	status = UrtIntSta(pADI_UART);

	if (status & COMIIR_NINT)
		return;   /* Check if UART is busy */

	/* Check what command to execute */
	switch (status & COMIIR_STA_MSK) {
	/* Check if UART register is available to be read */
	case COMIIR_STA_RXBUFFULL:
		/* Read character from UART */
		UART_ReadChar(&c);

		/* Check if read character is ENTER */
		if( c == 'y') {
			/* Set flag */
			uart_cmd = UART_TRUE;
			uart_received = UART_TRUE;
		} else {
			if ( c == 'n') {
				uart_cmd = UART_FALSE;
				uart_received = UART_TRUE;
			}
		}
		if (c == ENTER_KEY) {
			//enter received
			uart_received = UART_TRUE;
			uart_enter = UART_TRUE;
			s[_index] = '\0';
			_index = 0;
		}
		if (((c >= '0') && (c <= '9')) || (c == '.')) {
			s[_index] = c;
			_index++;
		}
		break;

	/* Check if UART register is available to be written */
	case COMIIR_STA_TXBUFEMPTY:

		/* Check uart counter */
		if (uart_tcnt) {

			/* UART is busy with writing*/
			uart_tbusy = UART_TRUE;

			/* Decrease  uart counter */
			uart_tcnt--;

			/* Write character to UART */
			UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);

			/* Check if TX buffer is full */
			if (uart_tpos == UART_TX_BUFFER_SIZE)
				/* Reset buffer counter  */
				uart_tpos = 0;

		} else
			/* UART is no longer busy with writing */
			uart_tbusy = UART_FALSE;

		break;

	default:
		break;
	}
}

/**
  @brief I2C0 Interrupt Handler

  @return none

**/
void I2C0_Master_Int_Handler(void)
{
	ui16I2cStatus = 0;
	ui16I2cStatus = I2cSta(MASTER);

	if((ui16I2cStatus & I2CMSTA_RXREQ) == I2CMSTA_RXREQ) // Master Receive IRQ
		uiMasterRxDat[uiMasterRxIndex++] = I2cRx(MASTER);
}


int main(int argc, char* argv[])
{
	timer_start ();

	/* Initialize UART port */
	UART_Init (B9600, COMLCR_WLS_8BITS);
	/* Initialize I2C communication */
	I2C_Init();

	AppPrintf("Test for turbidity\n\r");
	AppPrintf("Device id for ADPD105: %04x\n\r",
		  ADPD105_ReadReg(ADPD105_DEVID));

	/* Init and configure CN0409 */
	CN0409_Init();
	CN0409_AFECalibration();
	CN0409_ChannelOffsetCalibration();

	while (1) {
		CN0409_InteractiveMenu();

		timer_sleep(3000);
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
