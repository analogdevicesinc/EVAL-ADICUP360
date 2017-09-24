/*!
 *****************************************************************************
 * @file:    AD7793.c
 * @brief:   AD7793 converter
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

#include "Communication.h"
#include "AD7793.h"
#include "Timer.h"

/********************************* Global data ********************************/

const uint8_t reg_size[8] = {1, 2, 2, 3, 1, 1, 3, 3};


/**************************** Global functions *******************************/

/**
   @brief Initialization

   @return none
**/
void AD7793_Init(void)
{

   uint32_t ui32reg_value;


   SPI_Init();                                           /* SPI initialization */

   AD7793_Reset();                                       /* Reset ADC converter */

   ui32reg_value = (uint32_t) (AD7793_GAIN << 8);        /* Set ADC gain */
   ui32reg_value |= (uint32_t) AD7793_REFSEL;            /* Select internal reference source */
   ui32reg_value |= (uint32_t)AD7793_BUF;                /* Configure buffered mode of operation */

   AD7793_WriteRegister(AD7793_REG_CONF, ui32reg_value);  /* Set configuration options */

   AD7793_WriteRegister(AD7793_REG_IO, 0x02);           /* Set IOUT2 to 210 uA */

}

/**
   @brief ADC converter reset

   @return none
**/
void AD7793_Reset(void)
{

   SPI_Write(0, 0, 4);    /* Write 4 bytes = 0xFF */

   timer_sleep(0.05);   /* Wait time before accessing any registers after reset */
}


/**
   @brief Read register value

   @param ui8address - register address

   @return uint32_t - register value
**/
uint32_t AD7793_ReadRegister( uint8_t ui8address)
{

   static uint32_t ui32value;

   uint8_t ui8reg_adrr = (AD7793_COMM_READ | AD7793_COMM_ADR(ui8address));    /* Set value (read command + register address) to write in COMM register */

   ui32value = SPI_Read(ui8reg_adrr, reg_size[ui8address]);    /* Read register value */

   return ui32value;
}


/**
   @brief Write data to register

   @param ui8address - register address
   @param ui32data - data to write

   @return none
**/
void AD7793_WriteRegister( uint8_t ui8address, uint32_t ui32data)
{
   uint8_t ui8reg_adrr = (AD7793_COMM_WRITE | AD7793_COMM_ADR(ui8address));   /* Set value (write command + register address) to write in COMM register */

   SPI_Write(ui8reg_adrr, ui32data,  reg_size[ui8address]);  /* Write register value */

}


/**
   @brief Read ADC conversion results

   @param mode - conversion mode: SINGLE_CONV or CONTINUOUS_CONV
   @param ui8channel - ADC channel to scan

   @return uint32_t - conversion result
**/
uint32_t AD7793_Scan(enMode mode,  uint8_t ui8channel)
{
   static  uint32_t ui32result, ui32reg_value;

   uint8_t ui8reg_adrr = (AD7793_COMM_READ | AD7793_COMM_ADR(AD7793_REG_DATA));   /* Set value (read command + DATA register address) to write in COMM register */

   AD7793_SelectChannel(ui8channel);       /* Select channel to scan */

   if(mode == SINGLE_CONV) {                   /* Check if single conversion mode is wanted */

      ui32reg_value =  AD7793_ReadRegister(AD7793_REG_MODE);

      ui32reg_value &= AD7793_MODE_MSK;

      ui32reg_value |= (uint32_t)(mode << 13);      /* Set single mode operation */

      DioClr(CS_PORT, CS_PIN);

      AD7793_WriteRegister(AD7793_REG_MODE, ui32reg_value);

   }

   if(mode == CONTINUOUS_CONV) {

      DioClr(CS_PORT, CS_PIN);
   }

   while ((AD7793_ReadRegister(AD7793_REG_STAT)& RDY_BIT) != RDY_BIT);

   SPI_Write(0xAA, 0xAAAA, 2);

   ui32result =  SPI_Read(ui8reg_adrr,  reg_size[AD7793_REG_DATA]);

   DioSet(CS_PORT, CS_PIN);

   return ui32result;
}


/**
   @brief Select ADC input channel

   @param ui8channel - input channel

   @return none
**/
void AD7793_SelectChannel(uint8_t ui8channel)
{
   uint32_t ui32reg_value;

   ui32reg_value = AD7793_ReadRegister(AD7793_REG_CONF);     /* Read CONF register */
   ui32reg_value &= AD7793_CONF_MSK;
   ui32reg_value |= (uint32_t)ui8channel;                   /* Set set channel */
   AD7793_WriteRegister(AD7793_REG_CONF, ui32reg_value);    /* Write CONF register */

}

/**
   @brief Calibrate ADC input channel

   @param ui8channel - input channel
   @param mode - calibration mode: CAL_INT_ZERO_MODE, CAL_INT_FULL_MODE, CAL_SYS_ZERO_MODE, CAL_SYS_FULL_MODE

   @return none
**/
void AD7793_Calibrate(uint8_t ui8channel, enMode mode)
{
   uint32_t ui32reg_value, back_up;

   AD7793_SelectChannel(ui8channel);                  /* Select channel */

   ui32reg_value = back_up = AD7793_ReadRegister(AD7793_REG_MODE);        /* Read MODE register */

   ui32reg_value &= AD7793_MODE_MSK;

   ui32reg_value |= (uint32_t)(mode << 13);            /* Set mode */

   DioClr(CS_PORT, CS_PIN);

   AD7793_WriteRegister(AD7793_REG_MODE, ui32reg_value);          /* Write MODE register */

   while ((AD7793_ReadRegister(AD7793_REG_STAT)& RDY_BIT) != RDY_BIT);             /* Wait until RDY bit from STATUS register is high */

   DioSet(CS_PORT, CS_PIN);

}


/**
   @brief Convert ADC output into voltage

   @param u32adcValue - ADC code

   @return int32_t - converted voltage
**/
int32_t AD7793_ConvertToVolts(uint32_t u32adcValue)
{
   int32_t i32voltage;

   i32voltage = ((int64_t)(u32adcValue - 0x800000) * 1170) / (int32_t)0x800000; /* Vref = 1170 [mV]    */        /* Calculate voltage */

   return i32voltage;
}

