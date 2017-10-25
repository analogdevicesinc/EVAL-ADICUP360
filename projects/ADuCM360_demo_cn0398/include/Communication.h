/*!
 *****************************************************************************
 * @file:    Communication.cpp
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

#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include <adi_processor.h>
#include <SpiLib.h>
#include <DioLib.h>

#include <stddef.h>

#ifdef  __cplusplus
extern "C" {
#endif
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

/* CS_AD7124 - 0.3 - output */
#define CS_PORT      pADI_GP0
#define CS_PIN       0x08
#define CS_PIN_NUMBER   PIN3

/* ADP7118 - 1.2 - output */
#define ADP7118_PORT      pADI_GP1
#define ADP7118_PIN       0x04
#define ADP7118_PIN_NUMBER   PIN2

extern uint8_t convFlag;

extern char Rx_char;
extern bool read_ch;

extern unsigned char          uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern unsigned char          uart_tx_buffer[UART_TX_BUFFER_SIZE];

/****************************** Internal types *********************************/


/* Write data mode */
typedef enum {
   UART_WRITE_NO_INT = 1,            /* Write data when interrupts are disabled */
   UART_WRITE_IN_INT,               /* Write data while in an interrupt routine */
   UART_WRITE
} enWriteData;


extern unsigned char ucTxBufferEmpty;       // Used to indicate that the UART Tx buffer is empty
extern unsigned char ucWaitForUart;          // Used by calibration routines to wait for user input


#ifdef  __cplusplus
}
#endif // __cplusplus

class SPIClass {
public:

  static void Init();
  static void Write(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber);
  static void Read(unsigned char slaveDeviceId, unsigned char* data, unsigned char bytesNumber);

};

class UARTClass {
public:

  static void Init();
  static void WriteChar(char c);
  char ReadChar(void);

};


extern SPIClass SPI;
extern UARTClass UART;

#endif
