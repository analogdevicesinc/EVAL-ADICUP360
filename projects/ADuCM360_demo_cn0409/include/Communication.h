/**
******************************************************************************
*   @file     Communication.h
*   @brief    Header file for communication part
*   @version  V0.1
*   @author   ADI
*   @date     April 2017
*   @par Revision History:
*  - V0.1, April 2017: initial version.
*
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
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_


/* -------------------------------------------------------------------------*/
/* UART available modes */
#define UART_INT_MODE       1 /* Enables using both RX & TX interrupts */
#define UART_TX_INT_MODE      2 /* Enables using TX interrupts */
#define UART_RX_INT_MODE      3 /* Enables using RX interrupts */


/* The serial port may be used in polling or interrupt mode */
#define UART_MODE UART_TX_INT_MODE

#define ENTER_KEY	0x0D
/* -------------------------------------------------------------------------*/

/* Write data mode */
typedef enum {
	I2C_WRITE_ONE_REG = 1,           /* Write TEMP register */
	I2C_WRITE_TWO_REG,
	UART_WRITE_NO_INT,          /* Write data when interrupts are disabled */
	UART_WRITE_IN_INT,          /* Write data while in an interrupt routine */
	UART_WRITE
} enWriteData;

typedef enum {
	I2C_READ_ONE_REG = 1,            /* Read one TEMP register */
	I2C_READ_TWO_REG                 /* Read two TEMP registers */

} enRegsNum;

/* ------------------------------------------------------------------------- */

/* Execution status */
#define UART_SUCCESS             0
#define UART_FAILURE            -1
#define UART_NO_TX_SPACE        -2
#define UART_NO_RX_SPACE        -3

/* UART status */
#define UART_TRUE               1
#define UART_FALSE              0

/* Buffer size for UART Tx and Rx */
#define UART_TX_BUFFER_SIZE      1024       // UART transmit buffer size
#define UART_RX_BUFFER_SIZE      256        // UART receive buffer size

extern unsigned int			uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
extern unsigned int			uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;
extern unsigned int			uart_received, uart_enter;

extern unsigned char      uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern unsigned char      uart_tx_buffer[UART_TX_BUFFER_SIZE];

extern uint8_t volatile uiMasterRxIndex;
extern uint8_t volatile uiMasterTxIndex;
extern uint8_t volatile uiMasterTxDat[5];
extern uint8_t volatile uiMasterRxDat[5];

/* I2C pins */
#define I2C_PINS_0_12           5  /* Connected to P0.1, P0.2 */
#define I2C_PINS_2_01           6  /* Connected to P2.0, P2.1 */

/* Select I2C pins connection
 Available value:

    I2C_PINS_0_12 -> Connected to P0.1, P0.2
    I2C_PINS_2_01 -> Connected to P2.0, P2.1
*/
#define I2C_PINS     I2C_PINS_2_01

/* UART Functions */
void UART_Init(long lBaudrate, int iBits);
int UART_WriteChar(char data, enWriteData mode);
int UART_WriteString(char *string);
void UART_ReadChar(char *data);
void AppPrintf(const char *fmt, ...);

/*I2C Functions*/
void I2C_Init(void);
void I2C_Write(uint8_t ui8deviceAddress, uint8_t ui8regAddress, uint8_t ui8Data,
	       uint8_t ui8Data2, enWriteData enMode);
uint16_t I2C_Read(uint8_t ui8deviceAddress, uint8_t ui8regAddress,
		  enRegsNum enRegs);

#endif /* _COMMUNICATION_H_ */
