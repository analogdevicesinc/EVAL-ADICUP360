/*!
 *****************************************************************************
 * @file:    AD5270.c
 * @brief:   AD5270 rheostat
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

#include "AD5270.h"
#include "Timer.h"
#include "Communication.h"

/**
 * @brief Compute for the nearest RDAC value from given resistance
 * @param resistance - resistor
 * @return RDAC value - closest possible to given resistance
 */
uint16_t AD5270_CalcRDAC(float resistance)
{
   return ((uint16_t)((resistance / MAX_RESISTANCE) * 1024.0));

}
/**
 * @brief sets a new value for the RDAC
 * @param resistance new value for the resistance
 * @return actual value of the resistance in the RDAC
 */
float AD5270_WriteRDAC(float resistance)
{
   float RDAC_Value;

   uint16_t setValue;

    uint16_t RDAC_val = AD5270_CalcRDAC(resistance);

    RDAC_Value = (float)((RDAC_val * MAX_RESISTANCE) / 1024.0); // inverse operation to get actual resistance in the RDAC

    setValue = AD5270_ReadReg(READ_CTRL_REG);

    AD5270_WriteReg(WRITE_CTRL_REG, (setValue | RDAC_WRITE_PROTECT)); // RDAC register write protect -  allow update of wiper position through digital interface
    AD5270_WriteReg(WRITE_RDAC, RDAC_val); // write data to the RDAC register
    AD5270_WriteReg(WRITE_CTRL_REG, setValue); // RDAC register write protect -  allow update of wiper position through digital interface

    return RDAC_Value;
}

/**
 * Reads the RDAC register
 * @return RDAC resistor value
 */
float AD5270_ReadRDAC(void)
{
    uint16_t RDAC_val;

    RDAC_val = AD5270_ReadReg(READ_CTRL_REG);
    RDAC_val &= 0x03FF;

    return( ((float)(RDAC_val) * MAX_RESISTANCE) / 1024.0);
}

/**
 *	@brief Puts the AD5270 SDO line in to Hi-Z mode
 *	@return none
 */
void AD5270_Set_SDO_HiZ(void)
{
   uint8_t data1[2] = {HI_Zupper, HI_Zlower};
   uint8_t data2[2] = {NO_OP, NO_OP};

   DioClr(AD5270_CS_PORT, AD5270_CS_PIN);
   SPI_Write(data1, 2, AD5270);
   SPI_Write(data1, 2, AD5270);
   DioSet(AD5270_CS_PORT, AD5270_CS_PIN);

   DioClr(AD5270_CS_PORT, AD5270_CS_PIN);
   SPI_Write(data2, 2, AD5270);
   SPI_Write(data2, 2, AD5270);
   DioSet(AD5270_CS_PORT, AD5270_CS_PIN);

}



uint16_t AD5270_ReadReg(uint8_t command)
{

   uint8_t data[2];
   uint16_t result = 0;

   data[0] = (command & 0x3C);

   SPI_Read(data, 2, AD5270);

   result = data[0];
   result = (result << 8) | data[1];

   return result;
}
/**
 *  Enables the 50TP memory programming
 */
void AD5270_Enable_50TP_Programming(void)
{
    uint8_t regVal = (uint8_t)AD5270_ReadReg(READ_CTRL_REG);
    AD5270_WriteReg(WRITE_CTRL_REG, (regVal | PROGRAM_50TP_ENABLE)); // RDAC register write protect -  allow update of wiper position through digital interface
}

/**
 *  Stores current RDAC content to the 50TP memory
 */
void AD5270_Store_50TP(void)
{
   AD5270_WriteReg(STORE_50TP, 0);
   timer_sleep(WRITE_OPERATION_50TP_TIMEOUT);
}

/**
 * Disables the 50TP memory programming
 */
void AD5270_Disable_50TP_Programming(void)
{
    uint8_t regVal = AD5270_ReadReg(READ_CTRL_REG);
    AD5270_WriteReg(WRITE_CTRL_REG, (regVal & (~PROGRAM_50TP_ENABLE)));

}

/**
 * @brief Writes 16bit data to the AD5270 SPI interface
 * @param data to be written
 * @return data returned by the AD5270
 */
 void AD5270_WriteReg(uint8_t command, uint16_t value)
 {
      uint8_t data[2];

      data[0] = (command & 0x3C);
      data[0] |= (uint8_t)((value & 0x0300) >> 8);

      data[1] = (uint8_t)(value & 0x00FF);

      SPI_Write(data,2, AD5270);
 }

/**
 * Reads the last programmed value of the 50TP memory
 * @return last programmed value
 */
uint8_t AD5270_Read_50TP_LastAddress(void)
{
    uint8_t ret_val;

   AD5270_WriteReg(READ_50TP_ADDRESS,0);
   ret_val = AD5270_ReadReg(NO_OP);

   return ret_val;
}

/**
 * Reads the content of a 50TP memory address
 * @param address memory to be read
 * @return value stored in the 50TP address
 */
uint16_t AD5270_Read_50TP_memory(uint8_t address)
{
    uint16_t ret_val;

    AD5270_WriteReg(READ_50TP_CONTENTS, address);
    ret_val = AD5270_ReadReg(NO_OP);

    return ret_val;
}

/**
 * Resets the wiper register value to the data last written in the 50TP
 */
void AD5270_ResetRDAC(void)
{
   AD5270_WriteReg(SW_RST, 0);

}

/**
 * Changes the device mode, enabled or shutdown
 * @param mode - new mode of the device
 */
void AD5270_ChangeMode(AD5270Modes_t mode)
{

   AD5270_WriteReg(SW_SHUTDOWN, (uint16_t)(mode));
}


