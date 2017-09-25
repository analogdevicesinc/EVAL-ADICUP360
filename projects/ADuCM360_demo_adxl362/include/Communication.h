/*!
 *****************************************************************************
 * @file:    AD7798.h
 * @brief:   AD7798 Driver
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

/* LCD_CS_SEL pins */
#define CSLCD_PIN_P2_2           1   /* Select P2.2 */
#define CSLCD_PIN_P1_4           2   /* Select P1.4 */

/* ADXL_CS_SEL pins */
#define CSACC_PIN_P0_3           3   /* Select P0.3 */
#define CSACC_PIN_P0_4           4   /* Select P0.4 */

/* ADXL_INT_SEL pins */
#define INTACC_PIN_1             5   /* Select INT1 */
#define INTACC_PIN_2             6   /* Select INT2 */

/* LDC_RST_SEL pins */
#define RSLCD_PIN_IOREF           7   /* Select IOREF */
#define RSLCD_PIN_P1_1            8   /* Select P1.1 */

/*******************************************************************************
**************************** Functions declarations *****************************
********************************************************************************/

void SPI_Init(void);
void SPI_Write(uint8_t ui8address, uint8_t ui8Data, enWriteData enMode);
uint16_t SPI_Read(uint8_t ui8address, enRegsNum enRegs);


/*******************************************************************************
******************Configuration parameters(set by the user)*********************
********************************************************************************/
/* Select LCD CS pin based on jumper P8 setting.
 * Available values:
 *    CSLCD_PIN_P2_2
 *    CSLCD_PIN_P1_4 */
#define LCD_CS_SEL          CSLCD_PIN_P1_4

/* Select ADXL362 CS pin based on jumper P9 setting.
 * Available values:
 *    CSACC_PIN_P0_3
 *    CSACC_PIN_P0_4 */
#define ADXL_CS_SEL         CSACC_PIN_P0_4

/* Select ADXL362 INT pin based on jumper P7 setting.
 * Available values:
 *    INTACC_PIN_1
 *    INTACC_PIN_2 */
#define ADXL_INT_SEL        INTACC_PIN_1

/* Select LCD RST pin based on jumper P6 setting.
 * Available values:
 *    RSLCD_PIN_IOREF
 *    RSLCD_PIN_P1_1 */
#define LDC_RST_SEL         RSLCD_PIN_IOREF


/*********************Pins configuration (not set by the user)*******************/
/*** LCD CS pin configuration ***/
#if(LCD_CS_SEL == CSLCD_PIN_P1_4)
/* CSLCD - P1.4 - output */
#define CSLCD_PORT       pADI_GP1
#define CSLCD_PIN        0x10
#define CSLCD_PIN_NUMBER PIN4
#elif(LCD_CS_SEL == CSLCD_PIN_P2_2)
/* CSLCD - P2.2 - output */
#define CSLCD_PORT       pADI_GP2
#define CSLCD_PIN        0x04
#define CSLCD_PIN_NUMBER PIN2

#endif

/*** LCD A0 pin configuration ***/
/* A0 - P1.3 - output */
#define A0LCD_PORT         pADI_GP1
#define A0LCD_PIN          0x08
#define A0LCD_PIN_NUMBER   PIN3

/*** LCD BL pin configuration ***/
/* BL - P0.5 - output */
#define BLLCD_PORT         pADI_GP0
#define BLLCD_PIN          0x20
#define BLLCD_PIN_NUMBER   PIN5

/*** LCD RST pin configuration ***/
#if(LDC_RST_SEL == RSLCD_PIN_P1_1)
/* RES - P1.1 - output */
#define RSLCD_PORT        pADI_GP1
#define RSLCD_PIN         0x02
#define RSLCD_PIN_NUMBER  PIN1
#endif

/*** ACC CS pin configuration ***/
#if(ADXL_CS_SEL == CSACC_PIN_P0_4)
/* CSADXL362 - P0.4- output */
#define CSACC_PORT         pADI_GP0
#define CSACC_PIN          0x10
#define CSACC_PIN_NUMBER   PIN4
#elif(ADXL_CS_SEL == CSACC_PIN_P0_3)
/* CSADXL362 - P0.3- output */
#define CSACC_PORT         pADI_GP0
#define CSACC_PIN          0x08
#define CSACC_PIN_NUMBER   PIN3
#endif

/*** ACC INT pin configuration */
/* INT - P1.0 - input */
#define INTACC_PORT        pADI_GP1
#define INTACC_PIN         0x01
#define INTACC_PIN_NUMBER  PIN0

#endif /* _COMMUNICATION_H_ */
