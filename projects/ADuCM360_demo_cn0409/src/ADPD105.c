/**
******************************************************************************
*   @file     ADPD105.c
*   @brief    Source file for ADPD105 photometric front end.
*   @version  V0.1
*   @author   ADI
*   @date     April 2017
*   @par Revision History:
*  - V0.1, April 2017: initial version.
*
*******************************************************************************
* Copyright 2017(c) Analog Devices, Inc.
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


/***************************** Include Files **********************************/
#include <stdio.h>
#include <ADuCM360.h>
#include "ADPD105.h"
#include "Communication.h"
#include "Timer.h"


uint16_t au16DataSlotA[4] = {0,0,0,0};
uint16_t au16DataSlotB[4] = {0,0,0,0};

/************************* Functions Definitions ******************************/
/**
   @brief Writes a register to the ADPD105 via I2C.

   @param u8Address - register address
   @param u16value  - value to be written

   @return none

**/
void ADPD105_WriteReg(uint8_t u8Address, uint16_t u16Value)
{
	I2C_Write(ADPD105_ADDRESS, u8Address, (u16Value >> 8), (u16Value & 0xFF), I2C_WRITE_TWO_REG);
}

/**
   @brief Reads a register from ADPD105 via I2C.

   @param u8Address - register address

   @return uint16_t

**/
uint16_t ADPD105_ReadReg(uint8_t u8Address)
{
	return I2C_Read(ADPD105_ADDRESS, u8Address, I2C_READ_TWO_REG);
}

/**
   @brief Reads the DEVID register from ADPD105 via I2C.

   @return uint16_t

**/
uint16_t ADPD105_GetDevId(void)
{
	return ADPD105_ReadReg(ADPD105_DEVID);
}
/**
   @brief Reads the Status register from ADPD105 via I2C.

   @return uint16_t

**/
uint16_t ADPD105_GetStatus(void)
{
	return ADPD105_ReadReg(ADPD105_STATUS);
}

/**
   @brief Reads the Status register from ADPD105 via I2C.

   @param enMode - ADPD105 mode of operation

   @return uint16_t

**/
void ADPD105_SetOperationMode(enum ADPD105_OperationMode enMode)
{
	ADPD105_WriteReg(ADPD105_MODE, enMode);
}

/**
   @brief Configure Time Slot switch register from ADPD105 via I2C.

   @param u8SlotASelect - SlotA inputs

   @param u8SlotBSelect - SlotB inputs

   @return uint16_t

**/
void ADPD105_SetTimeSlotSwitch(enum ADPD105_TimeSlotPD u8SlotASelect,
			       enum ADPD105_TimeSlotPD u8SlotBSelect)
{
	uint16_t u16Value;

	u16Value = ADPD105_ReadReg(ADPD105_PD_LED_SELECT);
	/* reset before settings for PD Time slot */
	u16Value &= 0xF00F;
	/* set time slot values PD */
	u16Value |= ((uint16_t)u8SlotASelect << 4) |
		    (uint16_t)(u8SlotBSelect << 8);

	ADPD105_WriteReg(ADPD105_PD_LED_SELECT, u16Value);
}


/**
   @brief Enable the internal 32kHz internal clock from ADPD105 via I2C.

   @param u8SlotBSelect - u8Enable - enables/disables 32kHz clock

   @return void

**/
void ADPD105_SetCLK32K(uint8_t u8Enable)
{
	uint16_t u16Value;

	u16Value = ADPD105_ReadReg(ADPD105_SAMPLE_CLK);
	ADPD105_WriteReg(ADPD105_SAMPLE_CLK, u16Value |
			 (u8Enable << 7)); /* enable 32kHz internal clock */
}

/**
   @brief Setup FIFO for data reading from ADPD105.

   @return void

**/
void ADPD105_SetFIFO(void)
{
	ADPD105_SetOperationMode(PROGRAM);

	/* Slot A enable and 32 bit extend sample data to fifo */
	ADPD105_WriteReg(ADPD105_SLOT_EN, 0x3131);
	/* set FIFO threshold to 8 words data */
	ADPD105_WriteReg(ADPD105_FIFO_THRESH, 0x1F00);
	/* Set FIFO interrupt */
	ADPD105_WriteReg(ADPD105_INT_MASK, 0x0FF);
	ADPD105_WriteReg(ADPD105_GPIO_DRV, 0x05); /* set GPIO */

	ADPD105_SetOperationMode(NORMAL_OPERATION);
}

/**
   @brief Select what led to be used inside a time slot.

   @param enLEDNumber - the led number, see ADPD105_LED

   @param enSlot - time slot ( SlotA or SlotB)

   @return void

**/
void ADPD105_SelectLED(enum ADPD105_LED enLEDNumber,
		       enum ADPD105_TimeSlot enSlot)
{
	uint16_t u16Value;

	u16Value = ADPD105_ReadReg(ADPD105_PD_LED_SELECT);

	if (enSlot == SLOTA) {
		u16Value &= 0xFFFC;
		u16Value |= enLEDNumber; /* configure LED for SLOTA */
	} else {
		u16Value &= 0xFFF3;
		u16Value |= (enLEDNumber << 2); /* configure LED for SLOTB */
	}

	ADPD105_WriteReg(ADPD105_PD_LED_SELECT, u16Value);
}

/**
   @brief Deselect the leds for each time slot .

   @return void

**/
void ADPD105_DeselectLEDs(void)
{
	uint16_t u16Value;

	u16Value = ADPD105_ReadReg(ADPD105_PD_LED_SELECT);
	u16Value &= 0xFFF0;
	ADPD105_WriteReg(ADPD105_PD_LED_SELECT, u16Value);
}


/**
   @brief Read the data register for each time slot

   @return void

**/
void ADPD105_ReadDataRegs(uint16_t *data_Slot_A,
			  uint16_t *data_Slot_B, uint8_t count)
{
	uint8_t i;

	timer_sleep(30);

	/* Set data hold - disable data update */
	ADPD105_WriteReg(ADPD105_DATA_ACCESS_CTL, 0x07);

	/* Read data registers */
	for (i = 0; i < count; i++)
		data_Slot_A[i] = ADPD105_ReadReg(ADPD105_SLOTA_PD1_16BIT + i);

	for (i = 0; i < count; i++)
		data_Slot_B[i] = ADPD105_ReadReg(ADPD105_SLOTB_PD1_16BIT + i);

	/* disable data hold */
	ADPD105_WriteReg(ADPD105_DATA_ACCESS_CTL, 0x01);
}

/**
   @brief Display values of registers

   @return void

**/
void ADPD105_DisplayRegValues(uint16_t *data_slot_A,
			      uint16_t *data_slot_B, uint8_t data_count)
{
	uint8_t i;

	AppPrintf("SlotA Values\n\r");
	for ( i = 0; i < data_count; i++)
		AppPrintf("PD%d =  %d\n\r", i, data_slot_A[i]);

	AppPrintf("SlotB Values\n\r");
	for ( i = 0; i < data_count; i++)
		AppPrintf("PD%d =  %d\n\r", i, data_slot_B[i]);
}

/**
   @brief Set the width and offset for led pulse.

   @param enSlot - time slot (SlotA or SlotB)

   @param u8Width - the width of the led pulse (1us step)

   @param u8Offset - the offset of the led pulse (1us step)

   @return void

**/
void ADPD105_SetLEDWidthOffset(enum ADPD105_TimeSlot enSlot,
			       uint8_t u8Width, uint8_t u8Offset)
{
	uint16_t u16Value;

	if (enSlot == SLOTA) {
		u16Value = u8Offset + (uint16_t)((u8Width & 0x1F) << 8);
		ADPD105_WriteReg(ADPD105_SLOTA_LED_PULSE, u16Value);
	} else {
		u16Value = u8Offset + (uint16_t)((u8Width & 0x1F) << 8);
		ADPD105_WriteReg(ADPD105_SLOTB_LED_PULSE, u16Value);
	}
}

/**
   @brief Set the width and offset for AFE integration.

   @param enSlot - time slot (SlotA or SlotB)

   @param u8Width - the width of the AFE integration window (1us step)

   @param u8Offset - the offset of the AFE integration window (1us step)

   @param u8FineOffset - the fine offset of the AFE integration
   	   	   	   	   	   window (31.25 ns step)

   @return void

**/
void ADPD105_SetAFEWidthOffset(enum ADPD105_TimeSlot enSlot,
			       uint8_t u8Width, uint8_t u8Offset, uint8_t u8FineOffset)
{
	uint16_t u16Value = 0;

	if (enSlot == SLOTA) {
		u16Value = (uint16_t)(u8FineOffset & 0x1F) +
			   (uint16_t)((u8Offset & 0x3F) << 5) +
			   (uint16_t)((u8Width & 0x1F) << 11);
		ADPD105_WriteReg(ADPD105_SLOTA_AFE_WINDOW, u16Value);
	} else {
		u16Value = (uint16_t)(u8FineOffset & 0x1F) +
			   (uint16_t)((u8Offset & 0x3F) << 5) +
			   (uint16_t)((u8Width & 0x1F) << 11);
		ADPD105_WriteReg(ADPD105_SLOTB_AFE_WINDOW, u16Value);
	}
}


/**
   @brief Set the transimpendance amplifier gain.

   @param enSlot - time slot (SlotA or SlotB)

   @param enTIAGain - TIA gain value

   @return void

**/
void ADPD105_SetTIAGain(enum ADPD105_TimeSlot enSlot,
			enum ADPD105_TIAGain enTIAGain)
{
	uint16_t u16Value;

	if (enSlot == SLOTA) {
		u16Value = ADPD105_ReadReg(ADPD105_SLOTA_TIA_CFG);
		u16Value = 0x1C34 | enTIAGain;
		ADPD105_WriteReg(ADPD105_SLOTA_TIA_CFG, u16Value);
	} else {
		u16Value = ADPD105_ReadReg(ADPD105_SLOTB_TIA_CFG);
		u16Value = 0x1C34 | enTIAGain;
		ADPD105_WriteReg(ADPD105_SLOTB_TIA_CFG, u16Value);
	}
}

/**
   @brief Set the sampling frequency value.

   @param u16Frequency - sampling frequency value

   @return void

**/
void ADPD105_SetSamplingFrequency(uint16_t u16Frequency)
{
	uint16_t u16FValue;

	/* calculate value to write in FSAMPLE */
	u16FValue = 32000 / u16Frequency / 4;

	ADPD105_WriteReg(ADPD105_FSAMPLE, u16FValue);
}

/**
   @brief Set the value of the average factor N.

   @param enAverage - average factor value

   @return void

**/
void ADPD105_SetAverageFactor(enum ADPD105_AverageN enAverage)
{
	ADPD105_WriteReg(ADPD105_NUM_AVG, (enAverage << 4) +
			 (enAverage << 8));
}


/**
   @brief Set ADC clock speed.

   @param enADCClock - ADC clock speed value

   @return void

**/
void ADPD105_SetADCClock(enum ADPD105_ADCClockSpeed enADCClock)
{
	ADPD105_WriteReg(ADPD105_ADC_CLOCK, enADCClock);
}

/**
   @brief Enable digital clock

   @return void

**/
void ADPD105_SetDigitalClock(void)
{
	ADPD105_WriteReg(ADPD105_DATA_ACCESS_CTL, 1);
}

/**
   @brief Software reset for the ADPD105

   @return void

**/
void ADPD105_Reset(void)
{
	ADPD105_WriteReg(ADPD105_SW_RESET, 1); /* software reset */
}

/**
   @brief Set channel offset.

   @param enSlot - time slot (SlotA or SlotB)

   @param stOffset - offset value

   @return void

**/
void ADPD105_SetOffset(enum ADPD105_TimeSlot enSlot,
		       struct ADPD105_ChannelOffset stOffset)
{
	if (enSlot == SLOTA) {
		ADPD105_WriteReg(ADPD105_SLOTA_CH1_OFFSET, stOffset.CH1Offset);
		ADPD105_WriteReg(ADPD105_SLOTA_CH2_OFFSET, stOffset.CH2Offset);
		ADPD105_WriteReg(ADPD105_SLOTA_CH3_OFFSET, stOffset.CH3Offset);
		ADPD105_WriteReg(ADPD105_SLOTA_CH4_OFFSET, stOffset.CH4Offset);
	} else {
		ADPD105_WriteReg(ADPD105_SLOTB_CH1_OFFSET, stOffset.CH1Offset);
		ADPD105_WriteReg(ADPD105_SLOTB_CH2_OFFSET, stOffset.CH2Offset);
		ADPD105_WriteReg(ADPD105_SLOTB_CH3_OFFSET, stOffset.CH3Offset);
		ADPD105_WriteReg(ADPD105_SLOTB_CH4_OFFSET, stOffset.CH4Offset);
	}
}

/**
   @brief Disable the leds.

   @return void

**/
void ADPD105_DisableLed(void)
{
	ADPD105_WriteReg(ADPD105_LED_DISABLE, 0x300);
}

/**
   @brief Enable the leds.

   @return void

**/
void ADPD105_EnableLed(void)
{
	ADPD105_WriteReg(ADPD105_LED_DISABLE, 0);
}


/**
   @brief Set the pulse number and period.

   @param enSlot - time slot (SlotA or SlotB)

   @param u8PulseCount - number of pulses in time slot

   @param u8PulsePeriod - period of the pulse

   @return void

**/
void ADPD105_SetPulseNumberPeriod(enum ADPD105_TimeSlot enSlot,
				  uint8_t u8PulseCount, uint8_t u8PulsePeriod)
{
	uint16_t u16Value = ((uint16_t)u8PulseCount << 8) + u8PulsePeriod;

	if (enSlot == SLOTA) {
		ADPD105_WriteReg(ADPD105_SLOTA_NUMPULSES, u16Value);
	} else {
		ADPD105_WriteReg(ADPD105_SLOTB_NUMPULSES, u16Value);
	}
}
