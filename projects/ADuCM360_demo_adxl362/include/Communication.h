/**
******************************************************************************
*   @file     Communication.h
*   @brief    Header file for communication part
*   @version  V0.2
*   @author   ADI
*   @date     October 2015
*  @par Revision History:
*  - V0.1, September 2015: initial version.
*  - V0.2, October 2015: added missing comments and revision history.
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
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_


/*******************************************************************************
**************************** Internal types ************************************
********************************************************************************/

/* Write data mode */
typedef enum {
   SPI_WRITE_DATA = 1,            /* Write data to LCD */
   SPI_WRITE_COMMAND,               /* Write command to LCD */
   SPI_WRITE_REG                 /* Write ACC register */

} enWriteData;

typedef enum {
   SPI_READ_ONE_REG = 1,            /* Read one ACC register */
   SPI_READ_TWO_REG               /* Read two ACC registers */

} enRegsNum;


/*******************************************************************************
**************************** Internal definitions ******************************
********************************************************************************/

/* Accelerometer write command */
#define COMM_WRITE         0x0A

/* Accelerometer read command */
#define COMM_READ          0x0B

/* Unused address */
#define ADDR_NOT_USE       0x00


/*******************************************************************************
**************************** Functions declarations *****************************
********************************************************************************/

void SPI_Init(void);
void SPI_Write(uint8_t ui8address, uint8_t ui8Data, enWriteData enMode);
uint16_t SPI_Read(uint8_t ui8address, enRegsNum enRegs);


/*******************************************************************************
**************************** Configuration settings ****************************
********************************************************************************/

/*** LCD pin configuration ***/

/* CSLCD - P0.5 - output */
#define CSLCD_PORT      pADI_GP0
#define CSLCD_PIN       0x20

/* A0 - P1.1 - output */
#define A0LCD_PORT      pADI_GP1
#define A0LCD_PIN       0x02

/* BL - P1.0 - output */
#define BLLCD_PORT      pADI_GP1
#define BLLCD_PIN       0x01


/*** ACC pin configuration ***/

/* CSADXL362 - P2.2- output */
#define CSACC_PORT         pADI_GP2
#define CSACC_PIN          0x04
#define CSACC_PIN_NUMBER   0x02

/* INT - P0.4 - input */
#define INTACC_PORT        pADI_GP0
#define INTACC_PIN         0x10
#define INTACC_PIN_NUMBER  0x04

#define A0_BL_LCD_PORT      pADI_GP1


#endif /* _COMMUNICATION_H_ */
