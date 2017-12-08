/*!
 *****************************************************************************
 * @file:    CN0397.c
 * @brief:
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

#include "CN0397.h"
#include "Communication.h"

#include "ADuCM360.h"
#include "DioLib.h"
#include "UrtLib.h"

#include "AD7798.h"

#include "Timer.h"

#include <stdio.h>
#include <math.h>


bool _enter;   // enter key

uint8_t statusReg, idReg, ioReg, gainAdc;
uint16_t modeReg, configReg, offsetReg, fullscaleReg, dataReg;

uint16_t adcValue[3];
float voltageValue[3], intensityValue[3], lightConcentration[3];

int barLine[3];


const uint8_t Channels[] = { 1, 0, 2};

static const char *colour[] = {
   [0] = "RED",
   [1] = "GREEN",
   [2] = "BLUE",
};

const uint8_t ColorPrint[] = { 31, 32, 34 };

const uint8_t Gain[8] = { 1, 2, 4, 8, 16, 32, 64, 128};

const float Lux_LSB[3] = {2.122, 2.124, 2.113};

const float Optimal_Levels[3] = {26909.0, 8880.0, 26909.0};


void CN0397_DisplayData(void)
{

  uint8_t channel, i;

   for(channel = 0; channel < CHANNELS; channel++){

         printf("\t%s channel:[", colour[channel]);
         for(i = 0; i < barLine[channel] ; i++){
               printf("\033[2;%dm%c\033[0m",ColorPrint[channel], 219);
         }
         for(i = 0; i < (21 - barLine[channel]) ; i++){
               printf(" ");
         }
         printf("]");
         printf("\t");
   }

   printf("\n");
   printf("\t");

   for(channel = 0; channel < CHANNELS; channel++){

         printf("\tLight Intensity = %.2f lux\t\t", intensityValue[channel]);
   }

   printf("\n");
   printf("\t");

   for(channel = 0; channel < CHANNELS; channel++){

         printf("\tLight Concentration = %.2f%c\t\t", lightConcentration[channel], 37);
   }

   printf("\n");


   for(channel = 0; channel < 3; channel++){
         printf("\n");
   }

}


void CN0397_ReadADCData(uint8_t adcChannel, uint16_t *adcData)
{

   uint8_t channel;

   channel = 0x80 | adcChannel;

   convFlag = 1;

   DioClr(CS_PORT, CS_PIN);

   AD7798_SetRegisterValue(AD7798_REG_MODE, 0x200A, 2);

   while((AD7798_GetRegisterValue( AD7798_REG_STAT,1) & channel) != channel);

   timer_sleep(200);

   *adcData = AD7798_GetRegisterValue(AD7798_REG_DATA,2);

   DioSet(CS_PORT, CS_PIN);

   convFlag = 0;

}

void CN0397_ConvertToVoltage(uint16_t adcValue, float *voltage)
{

   *voltage = (float)(adcValue * V_REF)/(float)(_2_16 * gainAdc);

}

void CN0397_Init(void)
{
   uint8_t channel;

   SPI_Init();

   UART_Init(B115200, COMLCR_WLS_8BITS);

   AD7798_Reset();

   if(AD7798_Init()){

         AD7798_SetCodingMode(AD7798_UNIPOLAR);
         AD7798_SetMode(AD7798_MODE_SINGLE);
         AD7798_SetGain(ADC_GAIN);
         AD7798_SetFilter(ADC_SPS);
         AD7798_SetReference(AD7798_REFDET_ENA);

   }

   gainAdc = Gain[ADC_GAIN];

#if(USE_CALIBRATION == YES)
   printf("Calibrate the system:\n");
   printf("\n");

   for(channel = 0; channel < CHANNELS; channel++){

    _enter = false;
    printf("\tCalibrate %s channel: be sure that %s photodiode is cover and press <ENTER>.\n", colour[channel], colour[channel]);
    while(_enter == false);
    CN0397_Calibration(Channels[channel]);
    printf("\t\tChannel is calibrated!\n");
    printf("\n");

   }
   printf("System calibration complete!\n");
   printf("\n");
   printf("\n");

#endif
}

void CN0397_CalcLightIntensity(uint8_t channel, uint16_t adcValue, float *intensity)
{

   *intensity = adcValue * Lux_LSB[channel];

}

void CN0397_CalcLightConcentration(uint8_t channel, float intensity, float *conc)
{

   *conc = (intensity *100)/Optimal_Levels[channel];

}

void CN0397_SetAppData(void)
{
   uint8_t channel, rgbChannel;

   for(channel = 0; channel < CHANNELS; channel++){

      rgbChannel = Channels[channel];

      AD7798_SetChannel(channel);

      CN0397_ReadADCData(channel, &adcValue[rgbChannel]);
      CN0397_ConvertToVoltage(adcValue[rgbChannel], &voltageValue[rgbChannel]);
      CN0397_CalcLightIntensity(rgbChannel, adcValue[rgbChannel], &intensityValue[rgbChannel]);
      CN0397_CalcLightConcentration(rgbChannel, intensityValue[rgbChannel], &lightConcentration[rgbChannel]);
      CN0397_SetBar(lightConcentration[rgbChannel], &barLine[rgbChannel]);

   }
}

void CN0397_Calibration(uint8_t channel)
{

   uint16_t setValue;

   AD7798_SetChannel(channel);  //select channel to calibrate

   // Perform system zero-scale calibration
   setValue = AD7798_GetRegisterValue(AD7798_REG_MODE, 2);
   setValue &= ~(AD7798_MODE_SEL(0x07));
   setValue |= AD7798_MODE_SEL(AD7798_MODE_CAL_SYS_ZERO);
   AD7798_SetRegisterValue(AD7798_REG_MODE, setValue, 2);

   while((AD7798_GetRegisterValue( AD7798_REG_STAT,1) & channel) != channel);  // wait for RDY bit to go low

   while(AD7798_GetRegisterValue(AD7798_REG_MODE, 2) != 0x4005);    // wait for ADC to go in idle mode


}

void CN0397_SetBar(float conc, int *line)
{
   float concLimit = 5.0;
   int i = 0, j;
   *line = 0;

   if (conc > 0.0){
      i = 1;
      *line = i;
   }

   for(j = 0; j< 20; j++){
     if(conc >= concLimit){
            *line = i+1;
      }
      concLimit +=5.0;
      i +=1;
   }

}

