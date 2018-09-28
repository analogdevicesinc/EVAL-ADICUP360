/*
 *****************************************************************************
 * @file:    Communication.h
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
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <stdint.h>

/********************************* Internal defines ****************************/
#define UART_TX_BUFFER_SIZE      1024       // UART transmit buffer size
#define UART_RX_BUFFER_SIZE      256        // UART receive buffer size

/* Execution status */
#define UART_SUCCESS             0
#define UART_FAILURE            -1
#define UART_NO_TX_SPACE        -2
#define UART_NO_RX_SPACE        -3

/* UART status */
#define UART_TRUE                1
#define UART_FALSE               0

#define _CR                      13      /* <ENTER> */
#define _LF                      10      /* <New line> */
#define _SPC                     32      /* <Space> */
#define _BS                      8       /* <Backspace> */

#define READ_DATA_REG            0x58
/****************************** Global Data ***********************************/

extern unsigned int           uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
extern unsigned int           uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

extern unsigned char          uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern unsigned char          uart_tx_buffer[UART_TX_BUFFER_SIZE];

/****************************** Internal types *********************************/

char Rx_char;
uint8_t read_ch;

/* Number of received bytes */
typedef enum {
	ONE_BYTE = 1,                 /* Read one byte */
	TWO_BYTES                /* Read two bytes */
} enReadBytes;


/* Write data mode */
typedef enum {
	UART_WRITE_NO_INT = 1,            /* Write data when interrupts are disabled */
	UART_WRITE_IN_INT,               /* Write data while in an interrupt routine */
	UART_WRITE
} enWriteData;

/* SPI Definitions */

/* SPI Pins */

#define SPI1_PORT pADI_GP0

/* SPI Pins SCLCK/MISO/MOSI */

#define SPI1_SCLK_PIN 0x02
#define SPI1_SCLK_PIN_NUMBER PIN1
#define SPI1_MISO_PIN 0x01
#define SPI1_MISO_PIN_NUMBER PIN0
#define SPI1_MOSI_PIN 0x04
#define SPI1_MOSI_PIN_NUMBER PIN2

/* CS_AD7124 - 0.4 - output */

#define CS_AD7124_PIN 0x10
#define CS_AD7124_PIN_NUMBER PIN4

/* CS_AD5683 - 0.3 - output */

#define CS_AD5683_PIN 0x08
#define CS_AD5683_PIN_NUMBER PIN3

/* SPI Functions */

int32_t SPI_Init();
int32_t SPI_Write(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
		  uint8_t ui8_nr_bytes);
int32_t SPI_Read(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
		 uint8_t ui8_nr_bytes);
void UART_Init(long lBaudrate, int iBits);
void UART_ReadChar(char *data);
int UART_WriteChar(char data, enWriteData mode);
int UART_WriteString(char *string);
void AppPrintf(const char *fmt, ...);

#endif /* INCLUDE_COMMUNICATION_H_ */
