/**
******************************************************************************
*   @file     CN0409.c
*   @brief    Source file for CN0409 application
*   @version  V0.1
*   @author   ADI
*   @date     May 2017
*   @par Revision History:
*  - V0.1, May 2017: initial version.
*
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
*********************************************************************************/

/***************************** Include Files **********************************/
#include <stdio.h>
#include <ADuCM360.h>
#include <CN0409.h>
#include "ADPD105.h"
#include "Communication.h"
#include "math.h"
#include "Flash.h"
#include "Timer.h"
#include "stdlib.h"
#include "string.h"

/********************************** Variables **********************************/
float fpSlotAAverage[4] = {0,0,0,0};
float fpSlotBAverage[4] = {0,0,0,0};

float fpFirstPoint, fpSecondPoint;
float y_code[3], x_ntu[3], xm, ym, num, denom;

extern char s[100];

struct CN0409_ISORatioCalibrationCoeff  ISORatioCoeff;
struct CN0409_ISOCalibrationCoeff  ISONonRatioCoeff;
struct CN0409_GLICoeff  GLICoeff;

struct CN0409_NtuSolutions ntuSolutions;

double firstRatio, secondRatio;

/* Variables for GLI2 */
double firstSqrt, secondSqrt;

float fpTurbidityRatio, fpTurbidityNonRatio;

uint32_t  u32Buffer[5];

/************************* Functions Definitions ******************************/

/**
   @brief Initialize the ADPD105 and prepare for measurement

   @return void

**/
void CN0409_Init(void)
{
	/* Init CN0409 */
	ADPD105_SetOperationMode(PROGRAM); /* set programming mode */

	ADPD105_Reset();

	ADPD105_SetOperationMode(PROGRAM); /* set programming mode */

	/* enable 32k clock */
	ADPD105_SetCLK32K(1);
	/* select photodiodes 1-4 */
	ADPD105_SetTimeSlotSwitch(PD_1_4_CNNECTED, PD_1_4_CNNECTED);

	/* Select LEDs for each time slot */
	ADPD105_SelectLED(LEDX2, SLOTA);
	ADPD105_SelectLED(LEDX1, SLOTB);

	/* Set led course value */
	ADPD105_WriteReg(ADPD105_ILED3_COARSE, 0x1030);
	ADPD105_WriteReg(ADPD105_ILED1_COARSE, 0x3034);
	ADPD105_WriteReg(ADPD105_ILED2_COARSE, 0x3034);

	/* Set led fine value */
	/* 0x0208 - 80mA, 0x0451 - 100mA */
	ADPD105_WriteReg(ADPD105_ILED_FINE, 0x0451);

	/* Set pulse count */
	ADPD105_SetPulseNumberPeriod(SLOTA, 1, 0x18);
	ADPD105_SetPulseNumberPeriod(SLOTB, 1, 0x18);

	/* Set LED pulse width and offset */
	ADPD105_SetLEDWidthOffset(SLOTA, 2, 25);
	ADPD105_SetLEDWidthOffset(SLOTB, 2, 25);

	/* Set AFE width and offset */
	ADPD105_SetAFEWidthOffset(SLOTA, 3, 15, 0);
	ADPD105_SetAFEWidthOffset(SLOTB, 3, 15, 0);

	/* Average factor set to 8 */
	ADPD105_SetAverageFactor(AVERAGE8);

	/* Set ADC clock speed */
	ADPD105_SetADCClock(ADC_CLOCK_100MHz);

	/* Enable digital clock */
	ADPD105_SetDigitalClock();

	/* Set sampling frequency */
	/* 400 Hz sampling frequency */
	ADPD105_SetSamplingFrequency(400);

	/* Set TIA gain */
	ADPD105_SetTIAGain(SLOTA, TIA_200);
	ADPD105_SetTIAGain(SLOTB, TIA_200);

	/* Set sample clock */
	ADPD105_WriteReg(ADPD105_SAMPLE_CLK, 0x269a);
	ADPD105_WriteReg(ADPD105_CLK32M_ADJUST, 0x005e);
	ADPD105_WriteReg(ADPD105_EXT_SYNC_SEL, 0x20d0);
	ADPD105_WriteReg(ADPD105_CLK32M_CAL_EN, 0x0040);

	/* Set FIFO and start measurement */
	ADPD105_SetFIFO();
	ADPD105_SetOperationMode(NORMAL_OPERATION);
}

void CN0409_AFECalibration(void)
{
	/* offset test */
	struct ADPD105_ChannelOffset stOffset  = {4096, 4096, 4096, 4096};
	ADPD105_SetOffset(SLOTA, stOffset);
	ADPD105_SetOffset(SLOTB, stOffset);

	ADPD105_SetOperationMode(PROGRAM);
	ADPD105_EnableLed();

	/* Select LEDs for each time slot */
	ADPD105_SelectLED(LEDX2, SLOTA);
	ADPD105_SelectLED(LEDX1, SLOTB);

	/* Set pulse count */
	ADPD105_SetPulseNumberPeriod(SLOTA, 1, 0x18);
	ADPD105_SetPulseNumberPeriod(SLOTB, 1, 0x18);

	ADPD105_SetAFEWidthOffset(SLOTA, 3, 15, 0);
	ADPD105_SetAFEWidthOffset(SLOTB, 3, 15, 0);
	ADPD105_SetOperationMode(NORMAL_OPERATION);
}

/**
   @brief Do a channel offset calibration for ADPD105 (Channels 1 to 4)

   @return void

**/
void CN0409_ChannelOffsetCalibration(void)
{
	ADPD105_SetOperationMode(PROGRAM);

	ADPD105_DisableLed();

	struct ADPD105_ChannelOffset stOffset  = {0,0,0,0};
	ADPD105_SetOffset(SLOTA, stOffset);
	ADPD105_SetOffset(SLOTB, stOffset);

	/* Set pulse count */
	ADPD105_SetPulseNumberPeriod(SLOTA, 1, 0);
	ADPD105_SetPulseNumberPeriod(SLOTB, 1, 0);

	ADPD105_SetOperationMode(NORMAL_OPERATION);

	timer_sleep(3000);
	ADPD105_ReadDataRegs(au16DataSlotA, au16DataSlotB, 4);

	ADPD105_SetOperationMode(PROGRAM);

	/* Set offset for slot A */
	stOffset.CH1Offset = au16DataSlotA[0] - 4096;
	stOffset.CH2Offset = au16DataSlotA[1] - 4096;
	stOffset.CH3Offset = au16DataSlotA[2] - 4096;
	stOffset.CH4Offset = au16DataSlotA[3] - 4096;

	ADPD105_SetOffset(SLOTA, stOffset);

	/* Set offset for slot B */
	stOffset.CH1Offset = au16DataSlotB[0] - 4096;
	stOffset.CH2Offset = au16DataSlotB[1] - 4096;
	stOffset.CH3Offset = au16DataSlotB[2] - 4096;
	stOffset.CH4Offset = au16DataSlotB[3] - 4096;

	ADPD105_SetOffset(SLOTB, stOffset);

	ADPD105_EnableLed();

	/* Select LEDs for each time slot */
	ADPD105_SelectLED(LEDX2, SLOTA);
	ADPD105_SelectLED(LEDX1, SLOTB);

	/* Set pulse count */
	ADPD105_SetPulseNumberPeriod(SLOTA, 1, 0x18);
	ADPD105_SetPulseNumberPeriod(SLOTB, 1, 0x18);

	ADPD105_SetOperationMode(NORMAL_OPERATION);
}

/**
   @brief Tranform the codes read from ADPD105 to current values (in A)

   @return void

**/
void CN0409_CalculateRawData(void)
{
	uint8_t u8Count;
	uint16_t u16avg;

	timer_sleep(1000);
	/* Initialize globals to zero first */
	memset(fpSlotAAverage, 0, 4 * sizeof(float));
	memset(fpSlotBAverage, 0, 4 * sizeof(float));

	for(u16avg = 0; u16avg < (uint16_t)AVERAGE_DATA; u16avg++) {
		ADPD105_ReadDataRegs(au16DataSlotA, au16DataSlotB, 4);
		for (u8Count = 0; u8Count < 4; u8Count++) {
			/* Simple way of showing the board is
			 * performing an operation - Getting Data */
			if(u16avg % 64 == 0)
				AppPrintf("\rMeasuring data point.   ");
			if(u16avg % 64 == 15)
				AppPrintf("\rMeasuring data point..  ");
			if(u16avg % 64 == 31)
				AppPrintf("\rMeasuring data point... ");
			if(u16avg % 64 == 47)
				AppPrintf("\rMeasuring data point....");

			/* For now, don't convert to currents */
			fpSlotAAverage[u8Count] += (float)au16DataSlotA[u8Count] /
						   AVERAGE_DATA;
			fpSlotBAverage[u8Count] += (float)au16DataSlotB[u8Count] /
						   AVERAGE_DATA;
		}
	}
	AppPrintf("DONE!\n\r");
}

/**
   @brief User interface menu

   @return void

**/
void CN0409_InteractiveMenu(void)
{
	/* set to false the flag for receiving a char */
	uart_received = UART_FALSE;

	AppPrintf("\nSOLUTION TURBIDITY APPLICATION \n\n\r");

	CN0409_TurbidityCalculation();
}

/**
   @brief ISO7027 selection menu

   @return void

**/
void CN0409_TurbidityCalculation(void)
{
	float fpTemp;

	AppPrintf("Would you like to calibrate the device? (y - Yes, n - No)\n\r");
	while (!uart_received);
	uart_enter = 0;
	uart_received = 0;
	if (uart_cmd) {
		/* First solution is for the first point of Non-Ratio */
		AppPrintf(STRINGIFY1(Please place the FIRST_POINT
				     NTU sample and press enter!\n\r));
		while (!uart_received);
		uart_enter = 0;
		uart_received = 0;

		x_ntu[0] = FIRST_POINT;
		CN0409_CalculateRawData();
		fpFirstPoint = hypot(fpSlotAAverage[0], fpSlotBAverage[1]);

		/* Second solution is for the second point of Non-Ratio */
		AppPrintf(STRINGIFY1(Please place the SECOND_POINT
				     NTU sample and press enter!\n\r));
		while (!uart_received);
		uart_enter = 0;
		uart_received = 0;

		x_ntu[1] = SECOND_POINT;
		ntuSolutions.isoRatio1 = SECOND_POINT;
		CN0409_CalculateRawData();
		fpSecondPoint = hypot(fpSlotAAverage[0], fpSlotBAverage[1]);
		firstRatio = hypot((fpSlotAAverage[0] / fpSlotAAverage[1]),
				   (fpSlotBAverage[1] / fpSlotBAverage[0]));

		/* Third solution is for the second point of Ratio */
		AppPrintf(STRINGIFY1(Please place the THIRD_POINT
				     NTU sample and press enter!\n\r));
		while (!uart_received);
		uart_enter = 0;
		uart_received = 0;

		ntuSolutions.isoRatio2 = THIRD_POINT;
		CN0409_CalculateRawData();
		secondRatio = hypot((fpSlotAAverage[0] / fpSlotAAverage[1]),
				    (fpSlotBAverage[1] / fpSlotBAverage[0]));

		/* Calculate coefficients for RATIOMETRIC */
		CN0409_CalculateCoeff(RATIOMETRIC);
		u32Buffer[0] = *((uint32_t *) (&ISORatioCoeff.slope));
		u32Buffer[1] = *((uint32_t *) (&ISORatioCoeff.intercept));
		u32Buffer[2] = *((uint32_t *) (&ISORatioCoeff.status));
		FeePErs(RATIO_COEF_FLASH);
		WriteToFlash(u32Buffer, RATIO_COEF_FLASH, sizeof(u32Buffer));

		/*Calculate coefficients for NON-RATIOMETRIC*/
		CN0409_CalculateCoeff(NONRATIOMETRIC);
		u32Buffer[0] = *((uint32_t *) (&ISONonRatioCoeff.m));
		u32Buffer[1] = *((uint32_t *) (&ISONonRatioCoeff.b));
		u32Buffer[2] = *((uint32_t *) (&ISONonRatioCoeff.status));
		FeePErs(NONRATIO_COEF_FLASH);
		WriteToFlash(u32Buffer, NONRATIO_COEF_FLASH, sizeof(u32Buffer));
	} else {
		/*Load Calibration values from flash and calculate*/
		ReadFromFlash(u32Buffer, RATIO_COEF_FLASH, sizeof(u32Buffer));
		ISORatioCoeff.slope = *((float *)(&u32Buffer[0]));
		ISORatioCoeff.intercept = *((float *)(&u32Buffer[1]));
		ISORatioCoeff.status = (enum CN0409_CoeffUpdate)u32Buffer[2];

		ReadFromFlash(u32Buffer, NONRATIO_COEF_FLASH, sizeof(u32Buffer));
		ISONonRatioCoeff.m = *((float *)(&u32Buffer[0]));
		ISONonRatioCoeff.b = *((float *)(&u32Buffer[1]));
		ISONonRatioCoeff.status = (enum CN0409_CoeffUpdate)u32Buffer[2];

		if (ISORatioCoeff.status != UPDATED ||
		    ISONonRatioCoeff.status != UPDATED)
			/* Calibration is required */
			AppPrintf("Calibration is required because of "
				  "invalid coefficients! \n\r");
	}
	/* At this point, NTU measurements are auto-ranging */
	AppPrintf("Place the solution required to calculate "
		  "the turbidity value and press Enter!\n\r");
	while (!uart_received);
	uart_enter = 0;
	uart_received = 0;

	CN0409_CalculateRawData();

	if (ISORatioCoeff.status != UPDATED || ISONonRatioCoeff.status != UPDATED)
		/* Calibration is required */
		AppPrintf("Calibration is required because of "
			  "invalid coefficients! \n\r");
	else {
		/* NONRATIOMETRIC - Always run first */
		if(ISONonRatioCoeff.status == UPDATED) {
			fpTemp = hypot(fpSlotAAverage[0], fpSlotBAverage[1]);
			fpTurbidityNonRatio = (float)(fpTemp * ISONonRatioCoeff.m) +
					      ISONonRatioCoeff.b;
		}

		if(fpTurbidityNonRatio > 150) {
			/* RATIOMETRIC */
			if (ISORatioCoeff.status == UPDATED) {
				fpTemp = hypot((fpSlotAAverage[0] / fpSlotAAverage[1]),
					       (fpSlotBAverage[1] / fpSlotBAverage[0]));
				fpTurbidityRatio = (ISORatioCoeff.slope * fpTemp) +
						   ISORatioCoeff.intercept;
				AppPrintf("Turbidity value using Ratio is: %f \n\r",
					  fpTurbidityRatio);
			}
		} else
			AppPrintf("Turbidity value using Non-Ratio is: %f \n\r",
				  fpTurbidityNonRatio);
	}
}

/**
   @brief Calculate the coefficients for ISO7027

   @param mode - select between ratiometric or nonratiometric method

   @return void

**/
void CN0409_CalculateCoeff(enum CN0409_Type mode)
{
	switch(mode) {
	case RATIOMETRIC:
		ISORatioCoeff.status = UPDATED;

		ISORatioCoeff.slope = (float)((ntuSolutions.isoRatio2 -
					       ntuSolutions.isoRatio1)
					      / (secondRatio - firstRatio));
		ISORatioCoeff.intercept = (float)(ntuSolutions.isoRatio2 -
						  (secondRatio * ISORatioCoeff.slope));
		break;
	case NONRATIOMETRIC:
		num = 0;
		denom = 0;

		ISONonRatioCoeff.m = (x_ntu[1] - x_ntu[0]) /
				     (fpSecondPoint - fpFirstPoint);
		ISONonRatioCoeff.b = x_ntu[1] - ISONonRatioCoeff.m * fpSecondPoint;
		ISONonRatioCoeff.status = UPDATED;
		break;
	case GLI:
		GLICoeff.calSlope = (ntuSolutions.Gli2 - ntuSolutions.Gli1) /
				    (secondSqrt - firstSqrt);
		GLICoeff.cal0 = ntuSolutions.Gli2 - (GLICoeff.calSlope * secondSqrt);
		GLICoeff.status = UPDATED;
		break;
	default:
		break;
	}
}
