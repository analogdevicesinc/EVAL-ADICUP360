/*!
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

   /* UART Definitions */
   #define DEMO_BAUD_RATE     B115200
   #define DEMO_BIT_SIZE      COMLCR_WLS_8BITS

   #define _CR                13
   #define _LF                10
   #define _SPC               32
   #define _BS                8

   #define UART_RX_BUFFER_SIZE   256
   #define UART_TRUE             1
   #define UART_FALSE            0
   #define UART_SUCCESS          0

   /* SPI Definitions */
   #define CN0397_CS_PORT  pADI_GP0
   #define CN0397_CS_PIN   PIN3
   #define CN0397_CS_BIT   BIT3

   #define CN0398_CS_PORT  pADI_GP0
   #define CN0398_CS_PIN   PIN4
   #define CN0398_CS_BIT   BIT4

   #define RED_LED_CS_PORT    pADI_GP1
   #define RED_LED_CS_PIN     PIN0
   #define RED_LED_CS_BIT     BIT0

   #define BLE_LED_CS_PORT    pADI_GP1
   #define BLE_LED_CS_PIN     PIN1
   #define BLE_LED_CS_BIT     BIT1

   #define GRN_LED_CS_PORT    pADI_GP2
   #define GRN_LED_CS_PIN     PIN2
   #define GRN_LED_CS_BIT     BIT2

   #define RED_LED            0
   #define BLE_LED            2
   #define GRN_LED            1

   #define CN0397_SLAVE       0
   #define CN0398_SLAVE       1

   #define ADP7118_PORT       pADI_GP1
   #define ADP7118_BIT        0x04
   #define ADP7118_PIN        PIN2

   /* I2C Definitions */
   #define I2C_PRINT_RAW      1

   extern void Comms_Init(void);

   extern void UART_Init(long lBaudrate, int iBits);
   extern void UART_WriteChar(char c);
   extern char UART_ReadChar(void);
   extern int _write (int fd, char *ptr, int len);

   extern void SPI_Init(void);
   extern void SPI1_Disable(void);
   extern void SPI1_Enable(void);
   extern void SPI0_Write(uint16_t data, unsigned char channel);
   extern void SPI1_Write(unsigned char slaveDeviceId, unsigned char *data, unsigned char bytesNumber);
   extern void SPI1_Read(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber);

   extern void SPI_Write(unsigned char* data, unsigned char bytesNumber);
   extern void SPI_Read(unsigned char* data, unsigned char bytesNumber);

   extern void I2C_Init(void);
   extern uint8_t I2C_Write(uint8_t *ui8Data, uint8_t NumBytes, uint8_t address);
   extern uint8_t I2C_Read(uint8_t NumBytes, uint8_t address);

   extern uint8_t spi1TxComplete;
   extern uint8_t spi1RxComplete;

   extern uint8_t spi0TxComplete;
   extern uint8_t convFlag;

   extern uint8_t rxI2C;
   extern uint8_t txI2C;

   extern uint8_t rxI2Csize;
   extern uint8_t txI2Csize;

   extern uint8_t *rxI2Cbuf;
   extern uint8_t *txI2Cbuf;

   extern uint8_t rxI2Ccomplete;
   extern uint8_t txI2Ccomplete;

   extern uint8_t uart_rcnt;
   extern uint8_t uart_cmd;
   extern uint8_t uart_rdy;
   extern uint8_t uart_read_ch;
   extern char uart_rx_char;
   extern unsigned char uart_rx_buffer[UART_RX_BUFFER_SIZE];

#endif
