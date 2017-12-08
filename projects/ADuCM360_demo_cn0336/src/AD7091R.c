/*!
 *****************************************************************************
 * @file:    AD7091R.c
 * @brief:   AD7091R converter
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

#include <ADuCM360.h>
#include <DioLib.h>
#include <SpiLib.h>

#include "AD7091R.h"
#include "Communication.h"
#include "Timer.h"

/********************************* Global data ********************************/

uint8_t u8StartCounter;                                  /* Start counter variable */

/************************* Static functions prototypes ************************/

static void AD7091R_SoftwareReset(void);


/************************* Global functions *****************************/

/**
   @brief Initialization

   @return none
**/
void AD7091R_Init(void)
{
   SPI_Init();                                           /* SPI initialization */
   DioPulPin(CONVST_PORT, CONVST_PIN_NUMBER, 0);         /* Disable the internal pull up on CONVST pin */
   DioOenPin(CONVST_PORT, CONVST_PIN_NUMBER, 1);         /* Set CONVST pin as output */
   DioSet(CONVST_PORT, CONVST_PIN);                      /* Set CONVST pin as high */

   AD7091R_SoftwareReset();                             /* Perform an AD7091R software reset required at initialization */

   u8StartCounter = 0;                                  /* Reset start counter  */

}


/**
   @brief Software reset routine

   @return none
**/
static void AD7091R_SoftwareReset(void)
{

   CONVST_PORT->GPCLR = CONVST_PIN;                   /* Start conversion -> CONVST low */
   CONVST_PORT->GPSET = CONVST_PIN;                   /* Pull CONVST high */

   SPI_Read(AD7091R_OPERATION_MODE, ONE_BYTE);        /* Read a dummy conversion result */

}


/**
   @brief ADC conversion

   @return uint16_t - conversion result

**/
uint16_t AD7091R_Scan(void)
{

   uint16_t u16conversionResult;                     /* Define conversion result variable */


   timer_sleep(SCAN_TIME);                           /* Perform a delay */

   u16conversionResult = SPI_Read(AD7091R_OPERATION_MODE, TWO_BYTES);     /* Read conversion result */

   if(AD7091R_OPERATION_MODE == POWER_DOWN) {        /* Check if you are in power down mode */

      AD7091R_PowerUp();                           /* Perform power-up  */

   }

   return u16conversionResult;                       /* Return conversion result */


}


/**
   @brief Perform a power-up of the device -  keeps CONVST pin high for 50 ms

   @return uint16_t - conversion result

**/
void AD7091R_PowerUp(void)
{
   CONVST_PORT->GPSET = CONVST_PIN;             /* Pull CONVST high */

   timer_sleep(50);                             /* Wait 50 ms */

}


/**
   @brief Convert ADC value into voltage value

   @param u16adcValue  - ADC value
   @param f32VRef       - reference voltage

   @return float       - conversion result

**/
float AD7091R_ConvertToVolts(uint16_t u16adcValue, float f32VRef)
{
   float f32voltage = 0;

   if(f32VRef == 0) {                          /* Check if VRef is 0 */

      f32VRef = 2.5;                           /* Set to maximum Vref value */
   }

   f32voltage = f32VRef * (float)u16adcValue / ADC_RESOLUTION;           /* Calculate voltage value */

   return f32voltage;                          /* Return voltage value */
}

