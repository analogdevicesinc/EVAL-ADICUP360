/**
******************************************************************************
*   @file     Communication.h
*   @brief    Header file for communication part
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
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
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <stdio.h>
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


/****************************** Global Data ***********************************/


extern unsigned int           uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
extern unsigned int           uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

extern unsigned char          uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern unsigned char          uart_tx_buffer[UART_TX_BUFFER_SIZE];


extern unsigned char ucTxBufferEmpty;       // Used to indicate that the UART Tx buffer is empty
extern unsigned char ucWaitForUart;          // Used by calibration routines to wait for user input
/****************************** Internal types *********************************/

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


/*************************** Functions prototypes *****************************/
extern void UART_Init(long lBaudrate, int iBits);
extern char UART_ReadChar(void);
extern void UART_WriteChar(char c);
extern void SendString (void);
extern void UART_WriteString(char *data);
extern void UART_WriteStringNr(unsigned char data);

extern void SPI_Init(void);
extern void SPI_Write(unsigned char* data, unsigned char bytesNumber);
extern void SPI_Read(unsigned char* data, unsigned char bytesNumber);

extern uint8_t convFlag;

/**************************** Configuration parameters **********************/
//* CSAD7798 - 0.3 - output */
#define CS_PORT      pADI_GP0
#define CS_PIN       0x08
#define CS_PIN_NUMBER   PIN3

#endif /* _COMMUNICATION_H_ */
