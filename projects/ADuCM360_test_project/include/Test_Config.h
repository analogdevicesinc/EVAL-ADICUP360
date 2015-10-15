/**
******************************************************************************
*   @file     Test_Config.h
*   @brief    Header file for Test project
*   @version  V0.1
*   @author   ADI
*   @date     August 2015
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


#ifndef TEST_CONFIG_H_
#define TEST_CONFIG_H_

#include <stddef.h>
#include "Communication.h"
#include "Test.h"

/* Select which SPI channel do you want to test
 Available value:

   SPI_PMOD    -> SPI0
    SPI_ARDUINO -> SPI1

*/
#define SPI_CHANNEL  SPI_PMOD


/* Select UART pins connection
 Available value:

    UART_PINS_12 -> Connected to P0.1, P0.2
    UART_PINS_67 -> Connected to P0.6, P0.7
*/
#define UART_PINS    UART_PINS_12

/* Select I2C pins connection
 Available value:

    I2C_PINS_0_12 -> Connected to P0.1, P0.2
    I2C_PINS_2_01 -> Connected to P2.0, P2.1
*/
#define I2C_PINS     I2C_PINS_2_01

/* Select GPIO bank connection
 Available value:

   GPIO0 -> Connected from P0.0 to P0.7
   GPIO1 -> Connected from P1.0 to P1.7
   GPIO2 -> Connected from P2.0 to P2.2
*/
#define GPIO_PINS     GPIO0


/* Select what do you want to test
 Available value:

    UART
    SPI
    I2C
    GPIO
*/
#define FUNCTION_TO_TEST  GPIO


#endif /* TEST_CONFIG_H_ */
