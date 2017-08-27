/**
******************************************************************************
*   @file     AD7791.c
*   @brief    Source file for AD7791 converter
*   @version  V0.1
*   @author   ADI
*   @date     November 2015
*   @par Revision History:
*  - V0.1, November 2015: initial version.
*
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
*********************************************************************************/

#include <ADuCM360.h>
#include <SpiLib.h>
#include <DioLib.h>

#include "AD7791.h"
#include "Communication.h"
#include "Timer.h"
#include "math.h"


/**
   @brief Initializes ADC

   @return none

**/
void AD7791_Init (void)
{
   DioPulPin(CSAD7791_PORT, PIN5, 0);         /* Disable the internal pull up on CS pin */
   DioOenPin(CSAD7791_PORT, PIN5, 1);         /* Set CS pin as output */

   SPI_Init();                                        /*  Initialize SPI1 Bus */
   AD7791_Reset();                                    /* Reset the converter */

   SPI_Write(MODE_WRITE, 0x00, SPI_WRITE_ADC_REG);     /* Mode register value (single conversion, bipolar coding, unbuffered mode) */
   SPI_Write(FILTER_WRITE, 0x07, SPI_WRITE_ADC_REG);   /* Filter register value (clock not divided down, 9.5 Hz update rate) */
}


/**
   @brief Resets ADC to default status

   @return none

**/
void AD7791_Reset (void)
{
   SPI_Write(0x00, 0x00, SPI_WRITE_ADC_RESET);
   timer_sleep (1000);
}


/**
   @brief ADC conversion

   @return uint32_t - conversion result

**/
uint32_t AD7791_DataScan(void)
{

   uint32_t ui32conversionResult;

   ui32conversionResult = SPI_Read(DATA_READ , SPI_READ_ADC_DATA);    /* Read conversion result */

   return ui32conversionResult;                       /* Return conversion result */

}


/**
   @brief ADC register read

   @param ui8regValue  - Register id

   @return uint32_t - register read value

**/
uint32_t AD7791_RegRead(uint8_t ui8regValue)
{

   uint32_t ui32conversionResult;

   ui32conversionResult = SPI_Read(ui8regValue , SPI_READ_ADC_REG);    /* Read conversion result */

   return ui32conversionResult;                       /* Return conversion result */

}


/**
   @brief Convert ADC value into voltage value

   @param u32adcValue  - ADC value

   @return float       - conversion result

**/
float AD7791_ConvertToVolts(uint32_t ui32adcValue)
{
   float f32voltage = 0;

   f32voltage = VREF * ((float)ui32adcValue / (pow(2, 24)));          /* Calculate ADC input voltage value */

   return f32voltage;                          /* Return voltage value */
}
