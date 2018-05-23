/**
******************************************************************************
*   @file     ADPD105.c
*   @brief    Header file for ADPD105 photometric front end.
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

/*****************************************************************************/
/********************************  Definitions *******************************/
/*****************************************************************************/
/*Registers for ADPD105*/
#define ADPD105_STATUS             0x00
#define ADPD105_INT_MASK           0x01
#define ADPD105_GPIO_DRV           0x02
#define ADPD105_FIFO_THRESH        0x06
#define ADPD105_DEVID              0x08
#define ADPD105_I2CS_ID            0x09
#define ADPD105_CLK_RATIO          0x0A
#define ADPD105_GPIO_CTRL          0x0B
#define ADPD105_SLAVE_ ADDRESS_KEY 0x0D
#define ADPD105_SW_RESET           0x0F
#define ADPD105_MODE               0x10
#define ADPD105_SLOT_EN            0x11
#define ADPD105_FSAMPLE            0x12
#define ADPD105_PD_LED_SELECT      0x14
#define ADPD105_NUM_AVG            0x15
#define ADPD105_SLOTA_CH1_OFFSET   0x18
#define ADPD105_SLOTA_CH2_OFFSET   0x19
#define ADPD105_SLOTA_CH3_OFFSET   0x1A
#define ADPD105_SLOTA_CH4_OFFSET   0x1B
#define ADPD105_SLOTB_CH1_OFFSET   0x1E
#define ADPD105_SLOTB_CH2_OFFSET   0x1F
#define ADPD105_SLOTB_CH3_OFFSET   0x20
#define ADPD105_SLOTB_CH4_OFFSET   0x21
#define ADPD105_ILED3_COARSE       0x22
#define ADPD105_ILED1_COARSE       0x23
#define ADPD105_ILED2_COARSE       0x24
#define ADPD105_ILED_FINE          0x25
#define ADPD105_SLOTA_LED_PULSE    0x30
#define ADPD105_SLOTA_NUMPULSES    0x31
#define ADPD105_LED_DISABLE        0x34
#define ADPD105_SLOTB_LED_PULSE    0x35
#define ADPD105_SLOTB_NUMPULSES    0x36
#define ADPD105_ALT_PWR_DN         0x37
#define ADPD105_EXT_SYNC_STARTUP   0x38
#define ADPD105_SLOTA_AFE_WINDOW   0x39
#define ADPD105_SLOTB_AFE_WINDOW   0x3B
#define ADPD105_AFE_PWR_CFG1       0x3C
#define ADPD105_SLOTA_TIA_CFG      0x42
#define ADPD105_SLOTA_AFE_CFG      0x43
#define ADPD105_SLOTB_TIA_CFG      0x44
#define ADPD105_SLOTB_AFE_CFG      0x45
#define ADPD105_SAMPLE_CLK         0x4B
#define ADPD105_CLK32M_ADJUST      0x4D
#define ADPD105_ADC_CLOCK          0x4E
#define ADPD105_EXT_SYNC_SEL       0x4F
#define ADPD105_CLK32M_CAL_EN      0x50
#define ADPD105_AFE_PWR_CFG2       0x54
#define ADPD105_TIA_INDEP_GAIN     0x55
#define ADPD105_DIGITAL_INT_EN     0x58
#define ADPD105_DIG_INT_CFG        0x5A
#define ADPD105_DATA_ACCESS_CTL    0x5F
#define ADPD105_FIFO_ACCESS        0x60
#define ADPD105_SLOTA_PD1_16BIT    0x64
#define ADPD105_SLOTA_PD2_16BIT    0x65
#define ADPD105_SLOTA_PD3_16BIT    0x66
#define ADPD105_SLOTA_PD4_16BIT    0x67
#define ADPD105_SLOTB_PD1_16BIT    0x68
#define ADPD105_SLOTB_PD2_16BIT    0x69
#define ADPD105_SLOTB_PD3_16BIT    0x6A
#define ADPD105_SLOTB_PD4_16BIT    0x6B
#define ADPD105_A_PD1_LOW          0x70
#define ADPD105_A_PD2_LOW          0x71
#define ADPD105_A_PD3_LOW          0x72
#define ADPD105_A_PD4_LOW          0x73
#define ADPD105_A_PD1_HIGH         0x74
#define ADPD105_A_PD2_HIGH         0x75
#define ADPD105_A_PD3_HIGH         0x76
#define ADPD105_A_PD4_HIGH         0x77
#define ADPD105_B_PD1_LOW          0x78
#define ADPD105_B_PD2_LOW          0x79
#define ADPD105_B_PD3_LOW          0x7A
#define ADPD105_B_PD4_LOW          0x7B
#define ADPD105_B_PD1_HIGH         0x7C
#define ADPD105_B_PD2_HIGH         0x7D
#define ADPD105_B_PD3_HIGH         0x7E

/* Device address */
#define ADPD105_ADDRESS 0x64

/*Sensitivity*/
#define ADPD105_SENSITIVITY 1.64f

extern uint16_t au16DataSlotA[4];
extern uint16_t au16DataSlotB[4];

enum ADPD105_OperationMode {
	STANDBY = 0,
	PROGRAM,
	NORMAL_OPERATION
};


enum ADPD105_TimeSlotPD {
	ALL_INPUTS_FLOATING = 0,
	ALL_PD_CONNECTED,
	PD_5_8_CONNECTED = 0x04,
	PD_1_4_CNNECTED
};

enum ADPD105_TimeSlot {
	SLOTA = 0,
	SLOTB
};

enum ADPD105_LED {
	LEDX1 = 0x01,
	LEDX2,
	LEDx3
};

enum ADPD105_TIAGain {
	TIA_200 = 0,
	TIA_100,
	TIA_50,
	TIA_25
};

enum ADPD105_AverageN {
	AVERAGE1 = 0,
	AVERAGE2,
	AVERAGE4,
	AVERAGE8,
	AVERAGE16,
	AVERAGE32,
	AVERAGE64,
	AVERAGE128
};

enum ADPD105_ADCClockSpeed {
	ADC_CLOCK_100MHz = 0x40,
	ADC_CLOCK_500KHz = 0x60
};

struct ADPD105_ChannelOffset {
	uint16_t CH1Offset;
	uint16_t CH2Offset;
	uint16_t CH3Offset;
	uint16_t CH4Offset;
};

void ADPD105_WriteReg(uint8_t u8Address, uint16_t u16Value);
uint16_t ADPD105_ReadReg(uint8_t u8Address);
uint16_t ADPD105_GetDevId(void);
void ADPD105_SetOperationMode(enum ADPD105_OperationMode enMode);
void ADPD105_SetTimeSlotSwitch(uint8_t u8SlotASelect, uint8_t u8SlotBSelect);
void ADPD105_SetCLK32K(uint8_t u8Enable);
void ADPD105_SetFIFO(void);
void ADPD105_SelectLED(enum ADPD105_LED enLEDNumber,
		       enum ADPD105_TimeSlot enSlot);
void ADPD105_DeselectLEDs(void);
void ADPD105_ReadDataRegs(uint16_t *data_Slot_A, uint16_t *data_Slot_B,
			  uint8_t count);
void ADPD105_DisplayRegValues(uint16_t *data_slot_A, uint16_t *data_slot_B,
			      uint8_t data_count);
void ADPD105_SetLEDWidthOffset(enum ADPD105_TimeSlot enSlot, uint8_t u8Width,
			       uint8_t u8Offset);
void ADPD105_SetAFEWidthOffset(enum ADPD105_TimeSlot enSlot, uint8_t u8Width,
			       uint8_t u8Offset, uint8_t u8FineOffset);
void ADPD105_SetTIAGain(enum ADPD105_TimeSlot enSlot,
			enum ADPD105_TIAGain enTIAGain);
void ADPD105_SetSamplingFrequency(uint16_t u16Frequency);
void ADPD105_SetAverageFactor(enum ADPD105_AverageN enAverage);
void ADPD105_SetADCClock(enum ADPD105_ADCClockSpeed enADCClock);
void ADPD105_SetDigitalClock(void);
void ADPD105_Reset(void);
void ADPD105_SetOffset(enum ADPD105_TimeSlot enSlot,
		       struct ADPD105_ChannelOffset stOffset);
void ADPD105_DisableLed(void);
void ADPD105_EnableLed(void);
void ADPD105_SetPulseNumberPeriod(enum ADPD105_TimeSlot enSlot,
				  uint8_t u8PulseCount, uint8_t u8PulsePeriod);







