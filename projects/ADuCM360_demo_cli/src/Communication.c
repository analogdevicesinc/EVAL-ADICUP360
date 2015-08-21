/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for communication part.
*   @version  V0.1
*   @author   ADI
*   @date     August 2015
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
#include "Communication.h"
#include "ADuCM360.h"
#include "UrtLib.h"
#include "DioLib.h"



/************************** Variable Definitions ******************************/

unsigned char           uart_rx_buffer[UART_RX_BUFFER_SIZE];
unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int			uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
unsigned int			uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

/************************* Functions Definitions ******************************/

/**
	@brief UART initialization

**/
void UART_Init(long lBaudrate, int iBits)
{
	/*Configure UART pins */
#if(UART_PINS == UART_PINS_12)
	DioCfg(pADI_GP0, 0x003C);                    /* Configure P0.1/P0.2 for UART */
#elif(UART_PINS == UART_PINS_67)
	DioCfg(pADI_GP0, 0x9000);                    /* Configure P0.6/P0.7 for UART */
#endif

	UrtCfg(pADI_UART,lBaudrate,iBits,0); // Baud rate for 9600, 8-bits
	UrtMod(pADI_UART,COMMCR_DTR,0);             // <<<<< Modem Bits

	UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI);  /* Enables UART interrupt source */
	NVIC_EnableIRQ(UART_IRQn);                  // Enable UART IRQ
}

/**
	@brief Writes one character to UART.

	@param data - Character to write.
	@param mode - Write mode

	@return UART_SUCCESS or error code.

**/
int UART_WriteChar(char data, enWriteData mode)
{
	if(mode == UART_WRITE)
	{
		UrtTx(pADI_UART,data);

		return UART_SUCCESS;
	}
	else
	{
		if (uart_tcnt == UART_TX_BUFFER_SIZE){

			return UART_NO_TX_SPACE;

		} else{

			if (mode == UART_WRITE_NO_INT)
				NVIC_DisableIRQ(UART_IRQn);             /* Disable UART IRQ */

			if (uart_tbusy)	{
				uart_tx_buffer[(uart_tpos + (uart_tcnt++)) % UART_TX_BUFFER_SIZE] = data;
			} else {
				UrtTx(pADI_UART,data);
				uart_tbusy = UART_TRUE;
			}

			if (mode == UART_WRITE_NO_INT)
				NVIC_EnableIRQ(UART_IRQn);                  /* Enable UART IRQ */

			return UART_SUCCESS;
		}
	}
}

/**
	@brief Writes string to UART.

	@param string - string to write.

	@return UART_SUCCESS or error code.

**/
int UART_WriteString(char* string)
{
	int     result = UART_SUCCESS;

	while (*string!='\0') {
		result = UART_WriteChar(*string++, UART_WRITE_NO_INT);
		if (result != UART_SUCCESS)
			break;
	}

	return result;
}

/**
	@brief Read character from UART.

	@param data - data to read.

**/
void UART_ReadChar(char *data)
{
    *data = (char)UrtRx(pADI_UART);
}

