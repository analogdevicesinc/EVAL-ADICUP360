/*!
 *****************************************************************************
 * @file:    ADT7310.c
 * @brief:   ADT7310 Driver
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2016-2017 Analog Devices, Inc.

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

#include "ADT7310.h"
#include "Timer.h"
#include "Communication.h"


/******************************************************************************
 * @brief Resets ADT7310.
 *
 * @param None.
 *
 * @return None.
*******************************************************************************/
void ADT7310_Reset(void)
{
   uint8_t dataToSend[4] = {0xFF, 0xFF, 0xFF, 0xFF};

   SPI_Write(dataToSend,4, ADT7310);
   timer_sleep(1);

}

/******************************************************************************
 * @brief Reads ADT7310 register value.
 *
 * @param reg - which register to read.
 * @param size - register size.
 *
 * @return uint16_t - reading result.
*******************************************************************************/
uint16_t ADT7310_ReadReg(uint8_t reg, uint8_t size)
{
    uint8_t data[size];
    uint16_t result = 0;

    data[0] = ADT7310_READ | (reg << 3);

    SPI_Read(data, size, ADT7310);

    result = data[0];

    if(size > 1){

       for(uint8_t byte = 1; byte < size; byte++){
             result = (result << (byte * 8) | data[byte]);
       }
    }

    return result;

}

/******************************************************************************
 * @brief Writes an ADT7310 register.
 *
 * @param reg - in which register to write.
 * @param value - value to be written.
 * @param size - register size.
 *
 * @return None.
*******************************************************************************/
void ADT7310_WriteReg(uint8_t reg, uint16_t value, uint8_t size)
{
     uint8_t data[size + 1];
     uint8_t byte;
     uint16_t mask;

     data[0] = ADT7310_WRITE | (reg << 3);

     if(size == 1){

           mask = 0x00FF;

        } else{

           mask = 0xFF00;
        }

     for(byte = 1; byte <= size; byte++)
        {
           data[byte] = (uint8_t)((value & mask) >> ((size - byte) * 8));
           mask = mask >> (byte * 8);
        }

     SPI_Write(data,(1 + size), ADT7310);
}

/******************************************************************************
 * @brief Reads TEMP register and convert it into actual temperature.
 *
 * @param None.
 *
 * @return float - Temperature value.
*******************************************************************************/
float ADT7310_ReadTemp(void)
{
   uint16_t readData;
   float temp;

   readData = ADT7310_ReadReg(ADT7310_TEMP, 2);

   if(readData & 0x8000)
   {
     temp = (readData - 65536) / (128.0);
   }
   else
   {
     temp = readData / (128.0);
   }

   return temp;
}

/******************************************************************************
 * @brief Write setpoint value into setpoint registers.
 *
 * @param setpoint - which setpoint to write.
 * @param data - what value to write.
 *
 * @return None.
*******************************************************************************/
void ADT7310_WriteSetpoint(uint8_t setpoint, uint16_t data)
{
    ADT7310_WriteReg(setpoint, data, 2);
}

/******************************************************************************
 * @brief Start single conversion mode.
 *
 * @param None.
 *
 * @return None.
*******************************************************************************/
void ADT7310_StartSingleConversion(void)
{
    ADT7310_WriteReg(ADT7310_CONFIG, 0x20, 1);
}

