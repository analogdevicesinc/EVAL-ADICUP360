/***************************************************************************//**
 *   @file   AD7798.c
 *   @brief  Implementation of AD7798 Driver.
 *
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 577
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7798.h"				// AD7798 definitions.
#include "Communication.h"		// Communication definitions.
#include "Timer.h"
#include <stdio.h>
#include <stdlib.h>


/***************************************************************************//**
 * @brief Initializes the AD7798 and checks if the device is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char AD7798_Init(void)
{ 
	unsigned char status = 0x1;

   if ((AD7798_GetRegisterValue (AD7798_REG_ID, 1) & 0x0F) != AD7798_ID)
	{
		status = 0x0;
	}

	return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @param None.
 *
 * @return  None.    
*******************************************************************************/
void AD7798_Reset(void)
{
	unsigned char dataToSend[4] = {0xFF, 0xFF, 0xFF, 0xFF};

	SPI_Write(dataToSend, 4);

	timer_sleep(10);

}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
uint16_t AD7798_GetRegisterValue(unsigned char regAddress, unsigned char size)
{
	unsigned char *data = (unsigned char *)malloc(size * sizeof(unsigned char));
	uint16_t receivedData = 0x0000;
	unsigned char byte;

	data[0] = AD7798_COMM_READ |  AD7798_COMM_ADDR(regAddress);

	SPI_Read(data, size);

	receivedData = data[0];

	if(size > 1){

      for (byte = 1; byte < size; byte++)
      {
            receivedData = (receivedData << (byte * 8) | data[byte]);
      }
	}
	free((unsigned char *)data);
	return receivedData;

}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD7798_SetRegisterValue(uint8_t regAddress, uint16_t regValue,
                             uint8_t size)
{
	uint8_t *data = (uint8_t *)malloc((size + 1) * sizeof(uint8_t));
	uint8_t byte;
	uint16_t mask;

	data[0] = AD7798_COMM_WRITE |  AD7798_COMM_ADDR(regAddress);

	if(size == 1){

	      mask = 0x00FF;

	   } else{

	      mask = 0xFF00;
	   }

	for(byte = 1; byte <= size; byte++)
	   {
	      data[byte] = (uint8_t)((regValue & mask) >> ((size - byte) * 8));
         mask = mask >> (byte * 8);
	   }

	SPI_Write(data, (1 + size));
	free((uint8_t *)data);
}
/***************************************************************************//**
 * @brief Reads /RDY bit of status reg.
 *
 * @param None.
 *
 * @return rdy	- 0 if RDY is 1.
 *              - 1 if RDY is 0.
*******************************************************************************/
unsigned char AD7798_Ready(void)
{

    while((AD7798_GetRegisterValue( AD7798_REG_STAT,1) & 0x80) != 0x80);
	
	return(1);
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7798.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void AD7798_SetMode(unsigned long mode)
{
    unsigned long command;
    command = AD7798_GetRegisterValue(AD7798_REG_MODE,2);
    command &= ~AD7798_MODE_SEL(0xFF);
    command |= AD7798_MODE_SEL(mode);
    AD7798_SetRegisterValue(
            AD7798_REG_MODE,
            command,
            2
    );
}
/***************************************************************************//**
 * @brief Selects the channel of AD7798.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
void AD7798_SetChannel(unsigned long channel)
{
    unsigned long command;
    command = AD7798_GetRegisterValue(AD7798_REG_CONF,2);
    command &= ~AD7798_CONF_CHAN(0xFF);
    command |= AD7798_CONF_CHAN(channel);
    AD7798_SetRegisterValue(
            AD7798_REG_CONF,
            command,
            2
    );
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/
void AD7798_SetGain(uint16_t gain)
{
    uint16_t command;
    command = AD7798_GetRegisterValue(AD7798_REG_CONF,2);
    command &= ~AD7798_CONF_GAIN(0xFF);
    command |= AD7798_CONF_GAIN(gain);
    AD7798_SetRegisterValue(
            AD7798_REG_CONF,
            command,
            2
    );
}

void AD7798_SetFilter(unsigned int filter)
{
    unsigned long command;
    command = AD7798_GetRegisterValue(AD7798_REG_MODE,2);
    command &= ~AD7798_MODE_RATE(0x0F);
    command |= AD7798_MODE_RATE(filter);
    AD7798_SetRegisterValue(
            AD7798_REG_MODE,
            command,
            2
    );
}
/***************************************************************************//**
 * @brief Enables or disables the reference detect function.
 *
 * @param state - State of the reference detect function.
 *               Example: 0	- Reference detect disabled.
 *                        1	- Reference detect enabled.
 *
 * @return None.    
*******************************************************************************/
void AD7798_SetReference(unsigned char state)
{
    unsigned long command = 0;
    command = AD7798_GetRegisterValue(AD7798_REG_CONF,2);
    command &= ~AD7798_CONF_REFDET(1);
    command |= AD7798_CONF_REFDET(state);
    AD7798_SetRegisterValue(AD7798_REG_CONF,
							command,
							2);
}

void AD7798_SetCodingMode(uint8_t mode)
{
    uint16_t command;

    command = AD7798_GetRegisterValue(AD7798_REG_CONF,2);

    if(mode == AD7798_BIPOLAR){

          command &= ~AD7798_CONF_UNIPOLAR;

    } else if(mode == AD7798_UNIPOLAR){

          command |= AD7798_CONF_UNIPOLAR;
    }
    AD7798_SetRegisterValue(
            AD7798_REG_CONF,
            command,
            2
    );
}

void AD7798_SetBurnoutCurrent(uint8_t select)
{
    uint16_t command;

    command = AD7798_GetRegisterValue(AD7798_REG_CONF,2);

    if(select == AD7798_DISABLE)
       command &= ~AD7798_CONF_BO_EN;
    else if(select == AD7798_ENABLE)
          command |= AD7798_CONF_BO_EN;

    AD7798_SetRegisterValue(
            AD7798_REG_CONF,
            command,
            2
    );
}
