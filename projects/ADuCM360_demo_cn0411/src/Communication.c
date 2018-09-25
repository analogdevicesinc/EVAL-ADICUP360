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

#include <adi_processor.h>
#include <Communication.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <SpiLib.h>
#include <UrtLib.h>
#include <DioLib.h>

/********************************* Global data ********************************/


unsigned char           uart_rx_buffer[UART_RX_BUFFER_SIZE];
unsigned char           uart_tx_buffer[UART_TX_BUFFER_SIZE];

unsigned int         uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
unsigned int         uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

/**
   @brief SPI initialization

   @return none

**/

int32_t SPI_Init()
{
	int32_t ret = 0;

	DioPulPin(SPI1_PORT, SPI1_MISO_PIN_NUMBER, 0);
	DioCfgPin(SPI1_PORT, SPI1_MISO_PIN_NUMBER, 1);

	DioPulPin(SPI1_PORT, SPI1_MOSI_PIN_NUMBER, 0);
	DioCfgPin(SPI1_PORT, SPI1_MOSI_PIN_NUMBER, 1);

	DioPulPin(SPI1_PORT, SPI1_SCLK_PIN_NUMBER, 0);
	DioCfgPin(SPI1_PORT, SPI1_SCLK_PIN_NUMBER, 1);

	/* Disable the internal pull up on AD7124 CS pin */
	DioPulPin(SPI1_PORT, CS_AD7124_PIN_NUMBER, 0);

	/* Set CS pin for AD7124 as output */
	DioOenPin(SPI1_PORT, CS_AD7124_PIN_NUMBER, 1);

	/* Disable the internal pull up on AD5683 CS pin */
	DioPulPin(SPI1_PORT, CS_AD5683_PIN_NUMBER, 0);

	/* Set CS pin for AD5683 as output */
	DioOenPin(SPI1_PORT, CS_AD5683_PIN_NUMBER, 1);

	DioSet(SPI1_PORT, CS_AD7124_PIN);

	DioSet(SPI1_PORT, CS_AD5683_PIN);

	/* Set the SPI1 clock rate in Master mode to 1 MHz. */
	SpiBaud(pADI_SPI1, 3, SPIDIV_BCRST_DIS);

	/* Configure SPI1 - Clock polarity HIGH and SAMPLETRAILING */
	SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN,
	       SPICON_CON_EN | SPICON_SOEN_EN |
	       SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
	       SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN);

	return ret;

}

/**
   @brief Reads a specified register address in the converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - register address
   @param ui8_nr_bytes - register number of bytes

   @return reading result

**/

int32_t SPI_Read(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
		 uint8_t ui8_nr_bytes)
{

	int32_t ret = 0;
	/*Clear Slave based on ID */

	switch(ui8_slave_id) {
	case 0:
		SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN,
		       SPICON_CON_EN | SPICON_SOEN_EN |
		       SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
		       SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);

		DioClr(SPI1_PORT, CS_AD7124_PIN);
		break;
	case 1:
		SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN,
		       SPICON_CON_EN | SPICON_SOEN_EN |
		       SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
		       SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN);

		DioClr(SPI1_PORT, CS_AD5683_PIN);
		break;
	}

	/* Flush Tx and Rx FIFOs */
	SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);


	/* Send register address and dummy bytes in order to receive the register value */
	for (uint8_t ui8_counter = 0; ui8_counter < ui8_nr_bytes; ui8_counter++) {
		SpiTx(pADI_SPI1, ui8_buffer[ui8_counter]);
	}

	switch(ui8_nr_bytes-1) {
	case 1: {
		/*  Wait until 2 bytes are received */
		while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_TWOBYTES) !=
		       SPI1STA_RXFSTA_TWOBYTES);
		break;
	}
	case 2: {
		/*  Wait until 3 bytes are received */
		while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_THREEBYTES) !=
		       SPI1STA_RXFSTA_THREEBYTES);
		break;
	}
	case 3: {
		/*  Wait until 4 bytes are received */
		while ((SpiSta(pADI_SPI1) & SPI1STA_RXFSTA_FOURBYTES) !=
		       SPI1STA_RXFSTA_FOURBYTES);
		break;
	}
	}

	/* Read address and data bytes */
	for (uint8_t ui8_counter = 0; ui8_counter< ui8_nr_bytes; ui8_counter++) {
		ui8_buffer[ui8_counter] = SpiRx(pADI_SPI1);
	}

	/*Set Slave based on ID */
	switch(ui8_slave_id) {
	case 0:
		DioSet(SPI1_PORT, CS_AD7124_PIN);
		break;
	case 1:
		DioSet(SPI1_PORT, CS_AD5683_PIN);
		break;
	}

	if(ui8_nr_bytes == 0)
		ret = -1;

	return ret;

}

/**
   @brief Writes a register to the Converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - ACC register address
   @param ui32_data - value to be written
   @param ui8_nr_bytes - nr of bytes to be written

   @return none

**/

int32_t SPI_Write(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
		  uint8_t ui8_nr_bytes)
{
	int32_t ret = 0;

	if (ui8_nr_bytes > 4) {
		ui8_nr_bytes = 4;
	}

	/* Set FIFO status correct value */

	uint16_t ui16_fifo_status = (ui8_nr_bytes << 8);

	/*Clear Slave based on ID */
	switch(ui8_slave_id) {
	case 0:

		SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN,
		       SPICON_CON_EN | SPICON_SOEN_EN |
		       SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
		       SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);


		DioClr(SPI1_PORT, CS_AD7124_PIN);

		break;
	case 1:

		SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN,
		       SPICON_CON_EN | SPICON_SOEN_EN |
		       SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
		       SPICON_CPHA_SAMPLELEADING | SPICON_ENABLE_EN);

		DioClr(SPI1_PORT, CS_AD5683_PIN);

		break;
	}

	/* Flush Tx and Rx FIFOs */
	SpiFifoFlush(pADI_SPI1, SPICON_TFLUSH_EN, SPICON_RFLUSH_EN);

	/* Write address and data bytes */
	for (uint8_t ui8_counter = 0; ui8_counter < ui8_nr_bytes; ui8_counter++) {
		SpiTx(pADI_SPI1, ui8_buffer[ui8_counter]);
	}

	/* Wait until x bytes are received */
	while ((SpiSta(pADI_SPI1) & ui16_fifo_status) != ui16_fifo_status);

	/*Set Slave based on ID */
	switch(ui8_slave_id) {
	case 0:
		DioSet(SPI1_PORT, CS_AD7124_PIN);
		break;
	case 1:
		DioSet(SPI1_PORT, CS_AD5683_PIN);
		break;
	}

	if(ui8_nr_bytes == 0)
		ret = -1;

	return ret;

}

/**
  @brief UART initialization

  @param lBaudrate - Baud rate value (see UrtLib.c for values)
  @param iBits - Number of UART bits sent (see UrtLib.c for values)

  @return none

**/

void UART_Init(long lBaudrate, int iBits)
{

	DioCfgPin(pADI_GP0, PIN6, 1);          // P0.6 as UART RXD
	DioCfgPin(pADI_GP0, PIN7, 2);          // P0.7 as UART TXD
	UrtCfg(pADI_UART, lBaudrate, iBits, 0);      /* Configure UART bus */
	UrtMod(pADI_UART, COMMCR_DTR, 0);           /* Modem Bits */

	UrtIntCfg(pADI_UART, COMIEN_ERBFI |
		  COMIEN_ETBEI); /* Enables UART interrupt source */
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
		result = UART_WriteChar(*string++, UART_WRITE);

		if (result != UART_SUCCESS) {
			break;
		}
	}

	return result;
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

/**
  @brief Internal printf function with semihosting via UART.

  @param ptr - data to read.

  @return 1.

**/

int _read(int fd, char *ptr, int len)
{
	(void)fd;
	(void)len;

	while(read_ch == 0);

	*ptr = Rx_char;

	UART_WriteChar(*ptr, 1);
	printf("\n");

	read_ch = 0;

	return 1;
}





