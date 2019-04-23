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


#ifdef __cplusplus
extern "C"
{
#endif

/********************************* Internal defines ****************************/

#define UART_TX_QUEUE_SIZE 64
#define UART_RX_QUEUE_SIZE 64

#define MAX_BAUD_RATE 460800u
#define MIN_BAUD_RATE 19200u

#define CMD_BUFFER_SIZE 64

#define MSG_QUEUE_SIZE 64

/****************************** Global Data ***********************************/

extern volatile char uart_rx_queue[UART_RX_QUEUE_SIZE];
extern volatile int uart_rx_head, uart_rx_tail;
extern volatile char uart_tx_queue[UART_TX_QUEUE_SIZE];
extern volatile int uart_tx_head, uart_tx_tail;

extern char cmd_buffer[CMD_BUFFER_SIZE];
extern int cmd_head, cmd_tail;

/* Write data mode */
typedef enum {
	UART_WRITE_INT = 1,               /* Write data while in an interrupt routine */
	UART_WRITE
} enWriteData;

/*************************** Functions prototypes *****************************/
extern void UART_Init(long lBaudrate);
extern void UART_WriteChar(char c, enWriteData mode);
extern char UART_ReadChar(void);
extern void Cmd_ReadUART(void);

#ifdef __cplusplus
} //extern "C"
#endif

/**************************** Configuration parameters **********************/


#endif /* _COMMUNICATION_H_ */
