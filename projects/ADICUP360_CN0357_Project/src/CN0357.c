/*!
 *****************************************************************************
 * @file:    CN0357.c
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

/***************************** Include Files **********************************/

#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "UrtLib.h"
#include <ADuCM360.h>
#include "CN0357.h"
#include "AD7790.h"
#include "AD5270.h"
#include "Communication.h"
#include "Timer.h"

/********************************** Variables **********************************/

float fFeedbackResistor;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/************************* Functions Definitions ******************************/

/**
    @brief Initialize CN0357 Gas Detector Application

    @return none

**/
void CN0357_Init(void)
{
	/* Initialize SPI */
	SPI_Init();
}


/**
    @brief Calculate the Gas concentration in parts per million(PPM)

    @param fAdcVoltage  - value of ADC input voltage in volts
    @param fRDACValue   - value of the Rheostat resistance in ohms
    @param fsensitivity - sensitivity of the Gas sensor in amps/PPM

    @return value of the gas concentration in PPM

**/
float CN0357_CalcPPM(float fAdcVoltage, float fRDACValue, float fsensitivity)
{
	float fConcentration = 0;

	fConcentration = (fabs(fAdcVoltage) / fRDACValue) / fsensitivity;

	return fConcentration;
}

/**
    @brief Displays CN0357 circuit readings and data to the UART

    @param ui16Data - ADC data register value to be displayed
    @param fData1   - ADC input voltage reading to be displayed
    @param fdata2   - Gas Concentration reading to be displayed

**/
void CN0357_DisplayData(uint16_t ui16Data, float fData1, float fdata2)
{
	if (uart_cmd ==
	    UART_TRUE) {  /* condition becomes true when the system receives as Carriage Return(CR) command from the UART */

		AppPrintf("\r\nADC Data Register Value = %#08x",
			  ui16Data);   /* Send valid ADC data register value*/
		AppPrintf("\r\nADC Input Voltage input = %f V",
			  fData1);      /* Send valid voltage input value */
		AppPrintf("\r\nGas Concentration = %f PPM",
			  fdata2);          /* Send valid gas concentration value */

		UART_WriteString("\r\n");

		uart_cmd = UART_FALSE;
	}

}



