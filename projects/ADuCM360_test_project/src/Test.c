/*!
 *****************************************************************************
 * @file:    Test.c
 * @brief:   Test functions
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
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include <ADuCM360.h>
#include <DioLib.h>
#include <UrtLib.h>
#include <SpiLib.h>


#include "Test.h"
#include "Timer.h"
#include "Communication.h"
#include "Test_Config.h"



static void Test_SPI(void);
static void Test_UART(void);
static void Test_I2C(void);
static void Port_Init(void);

/************************* Functions Definitions ******************************/


/**
  @brief SPI test function

  @return none

**/
static void Test_SPI(void)
{
   if(SPI_CHANNEL == SPI_PMOD) { /* Check if SPI0 channel is tested */
      SPI_Write(pADI_SPI0, 0x03);    /* Write data to SPI0 */

   } else if (SPI_CHANNEL == SPI_ARDUINO) { /* Check if SPI1 channel is tested */

      SPI_Write(pADI_SPI1, 0x20);    /* Write data to SPI1 */
   }
}


/**
  @brief UART test function

  @return none

**/
static void Test_UART(void)
{

   if(UART_PINS == UART_PINS_67) { /* Check if UART is connected to P0.6-P0.7 pins */
      UART_WriteChar('V');  /* 0x56 - Write data to UART*/

   } else if(UART_PINS == UART_PINS_12) { /* Check if UART is connected to P0.1-P0.2 pins */
      UART_WriteChar(0x43);    /* Write data to UART */
   }

}

/**
  @brief I2C test function

  @return none

**/
static void Test_I2C(void)
{

   I2C_Write(0x01);   /* Write data to I2C */

}

/**
  @brief UART, SPI, I2C, GPIO initialization

  @return none

**/
void Test_Init(void)
{

   if(FUNCTION_TO_TEST == SPI) { /* Check if SPI is tested */
      SPI_Init();         /* SPI initialization */

   } else if(FUNCTION_TO_TEST == UART) { /* Check if UART is tested */
      UART_Init(B9600, COMLCR_WLS_8BITS);    /* UART initialization: 9600 baud rate and 8 bits data */

   } else if(FUNCTION_TO_TEST == I2C) { /* Check if I2C is tested */
      I2C_Init();         /* I2C initialization */

   } else if(FUNCTION_TO_TEST == GPIO) { /* Check if GPIO is tested */
      Port_Init();
   }

}

/**
  @brief Test peripherals function

  @return none

**/
void Test_Periph(void)
{
   if(FUNCTION_TO_TEST == SPI) { /* Check if SPI is tested */
      Test_SPI();             /* Test SPI */

   } else if(FUNCTION_TO_TEST == UART) { /* Check if UART is tested */
      Test_UART();            /* Test UART */

   } else if(FUNCTION_TO_TEST == I2C) { /* Check if I2C is tested */
      Test_I2C();            /* Test I2C */

   }

}

/**
  @brief Port initialization

  @return none

**/
static void Port_Init(void)
{
   if (GPIO_PINS == GPIO0) { /* Check if Port 0 is tested as GPIO */
      DioCfg(pADI_GP0, 0x4000);     /* Port 0 configured to GPIO */
      DioPul(pADI_GP0, 0);        /* Disables all pull-up resistors */
      DioOen(pADI_GP0, 0xFF);       /* Sets Port 0 to be outputs */
      DioClr(pADI_GP0, 0xFF);       /* Clear Port 0 outputs */

   } else if (GPIO_PINS == GPIO1) { /* Check if Port 1 is tested as GPIO */
      DioCfg(pADI_GP1, 0x0000);     /* Port 1 configured to GPIO */
      DioPul(pADI_GP1, 0);        /* Disables all pull-up resistors */
      DioOen(pADI_GP1, 0xFF);       /* Sets Port 1 to be outputs */
      DioClr(pADI_GP1, 0xFF);       /* Clear Port 1 outputs */

   } else if (GPIO_PINS == GPIO2) { /* Check if Port 2 is tested as GPIO */
      DioCfg(pADI_GP2, 0xAA80);     /* Port 2 configured to GPIO */
      DioPul(pADI_GP2, 0);        /* Disables all pull-up resistors */
      DioOen(pADI_GP2, 0x07);       /* Sets Port 2 to be outputs */
      DioClr(pADI_GP2, 0x07);       /* Clear Port 2 outputs */
   }
}


/**
  @brief Port test function

  @return none

**/
void Test_Port(void)
{
   if (GPIO_PINS == GPIO0) { /* Check if Port 0 is tested as GPIO */
      DioSet(pADI_GP0, 0xFF);  /* Set Port 0 outputs to High */

   } else if (GPIO_PINS == GPIO1) { /* Check if Port 1 is tested as GPIO */
      DioSet(pADI_GP1, 0xFF);   /* Set Port 1 outputs to High */

   } else if (GPIO_PINS == GPIO2) { /* Check if Port 2 is tested as GPIO */
      DioSet(pADI_GP2, 0x07);     /* Set Port 2 outputs to High */
   }

}

