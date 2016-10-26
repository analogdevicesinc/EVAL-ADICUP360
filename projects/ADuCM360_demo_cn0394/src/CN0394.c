/******************************************************************************
*   @file     CN0394.c
*   @brief    Source file for ADC handling
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
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
*******************************************************************************/

#include "CN0394.h"
#include "RTD.h"
#include "Thermocouple.h"
#include "Communication.h"

#include "ADuCM360.h"
#include "DioLib.h"
#include <AdcLib.h>
#include <IexcLib.h>

#include "Timer.h"

#include <stdio.h>
#include <stdbool.h>

#include <math.h>

#include "ADC.h"

static const unsigned char thermocouple_type[] = { 'T', 'J', 'K', 'E', 'S', 'R', 'N', 'B' };

float adc0_voltage[4], adc1_voltage[4], Rrtd_value[4], temp0[4], temp1[4];
int32_t adc0[4], adc1[4];

uint8_t th_types[4];

bool calibFlag0, calibFlag1;
uint8_t write0, write1;

void CN0394_DisplayData(void)
{

   cn0394_channel count = 0;

   for(count = CHANNEL_1; count <= CHANNEL_4; count++){

      printf("P%d channel (Type %c):\n", count+1, thermocouple_type[th_types[count]]);
      printf("\tR%d resistance = %f ohmi\n",count+1, Rrtd_value[count]);
      printf("\tRTD Temperature = %f°C\n",temp0[count]);
      printf("\tLinearized Temperature = %f°C\n",temp1[count]);

   }

 printf("\n");
 printf("\n");
}

void CN0394_SetData(void)
{
   cn0394_channel count;

   for(count = CHANNEL_1; count <= CHANNEL_4; count++){

        CN0394_ChannelConfig(RTD_CHANNEL, count);
        CN0394_ChannelConfig(TH_CHANNEL, count);

        CN0394_StartConversion(RTD_CHANNEL);
        CN0394_StartConversion(TH_CHANNEL);

        Rrtd_value[count] = CN0394_CalcRrtd(adc0_data);
        CN0394_Calc_RTDTemperature(count, &temp0[count]);

        CN0394_Calc_ThermocoupleTemperature(count, temp0[count], &temp1[count]);

   }

}

void CN0394_Init(void)
{

   ADC0INIT();                                                            // Init ADC0
   IEXCINIT();
   NVIC_EnableIRQ(ADC0_IRQn);
   ADC1INIT();                                                            // Init ADC1
   NVIC_EnableIRQ(ADC1_IRQn);                                         // ADC1 IRQ

   write0 = write1 = 0;
   calibFlag0 = calibFlag1 = false;

   th_types[CHANNEL_1] = P1_TYPE;
   th_types[CHANNEL_2] = P2_TYPE;
   th_types[CHANNEL_3] = P3_TYPE;
   th_types[CHANNEL_4] = P4_TYPE;

}


float CN0394_CalcVoltage(uint8_t channel, int32_t adcValue)
{
   float voltage;

   if(channel == RTD_CHANNEL){
         voltage = (VREF0/_2_28)*adcValue;
   } else{
         voltage = (VREF1/_2_28)*adcValue;
   }

   return voltage;
}

void CN0394_ChannelConfig(uint8_t channel, uint8_t adcInput)
{

   uint8_t input = 4 + adcInput;

   if(channel == RTD_CHANNEL){

         if(input == 7){
               AdcPin(pADI_ADC0,(input - 1), input << 5);
         } else{
               AdcPin(pADI_ADC0,(input + 1), input << 5);
         }
         IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Ext,input << 3,input);


   } else{

         AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN11,adcInput << 5);
   }

}
 float CN0394_CalcCurrent(float voltage)
 {
    float current;

    current = voltage/R5;

    return current;
 }

 float CN0394_CalcRrtd(int32_t adc)
 {
    float rRtd;

    rRtd = (adc/_2_28)*R5;

    return rRtd;
 }

void CN0394_Calc_RTDTemperature(cn0394_channel ch, float *temp)
 {
     float rRtd;

     rRtd = ((float)adc0[ch]/_2_28)*R5;


      if(rRtd > R0) {

        *temp = (-COEFF_A + sqrt(COEFF_A_A - COEFF_4B_R0*(R0 - rRtd)))/COEFF_2B;


      } else {

         POLY_CALC(*temp, rRtd/10.0, &cjPolyCoeff[0]);
     }

 }

void CN0394_StartConversion(uint8_t channel)
{
   if(channel == RTD_CHANNEL)
         AdcGo(pADI_ADC0,ADCMDE_ADCMD_SINGLE);
   else
         AdcGo(pADI_ADC1,ADCMDE_ADCMD_SINGLE);
   timer_sleep(100);

}

void CN0394_Calc_ThermocoupleTemperature(cn0394_channel ch, float cjTemp, float *buffer)
{
   float cjVoltage, thVoltage;
    const temp_range *thCoeff;

    thCoeff = &thPolyCoeff[th_types[ch]];

      thVoltage = (float)adc1[ch] * (VREF1/_2_28);

      if(cjTemp < thTempRange[th_types[ch]][1]) {
         POLY_CALC(cjVoltage, cjTemp, thCoeff->neg_temp);
      } else {

        if(cjTemp <= thTempRange[th_types[ch]][2]){

          POLY_CALC(cjVoltage, cjTemp, thCoeff->pos_temp1);
          if(th_types[ch] == TYPE_K){
             cjVoltage += COEFF_K_A0*exp(COEFF_K_A1*(cjTemp - COEFF_K_A2)*(cjTemp - COEFF_K_A2));
           }
        } else{
          POLY_CALC(cjVoltage, cjTemp, thCoeff->pos_temp2);
        }
      }

      thVoltage += cjVoltage;

      if(thVoltage < thVoltageRange[th_types[ch]][1]) {
        POLY_CALC(*buffer, thVoltage, thCoeff->neg_voltage);
      } else {
          if(thVoltage <= thVoltageRange[th_types[ch]][2]) {
            POLY_CALC(*buffer, thVoltage, thCoeff->pos_voltage1);
          }else{

             if(thVoltage <= thVoltageRange[th_types[ch]][3]) {
              POLY_CALC(*buffer, thVoltage, thCoeff->pos_voltage2);
            }else{
                  POLY_CALC(*buffer, thVoltage, thCoeff->pos_voltage3);
            }
          }
       }

}

void CN0394_Calibration(uint8_t channel)
{

   if(channel == RTD_CHANNEL){

      calibFlag0 = true;

      AdcGo(pADI_ADC0,ADCMDE_ADCMD_IDLE); //Set ADC channel in idle mode
      timer_sleep(1);

      AdcGo(pADI_ADC0,ADCMDE_ADCMD_INTOCAL); // ADC0 internal Zero scale calibration
      while ((AdcSta(pADI_ADC0) &0x20) != 0x20);       // bit 5 set by adc when calibration is complete

      AdcGo(pADI_ADC0,ADCMDE_ADCMD_INTGCAL); // ADC0 internal full scale calibration
      while ((AdcSta(pADI_ADC0) &0x20) != 0x20);       // bit 5 set by adc when calibration is complete

      AdcGo(pADI_ADC0,ADCMDE_ADCMD_IDLE); //Set ADC channel in idle mode
      timer_sleep(1);

      calibFlag0 = false;

   }
   else
      {
         calibFlag1 = true;

         AdcGo(pADI_ADC1,ADCMDE_ADCMD_IDLE); //Set ADC channel in idle mode
         timer_sleep(1);

         AdcGo(pADI_ADC1,ADCMDE_ADCMD_INTOCAL); // ADC1 internal Zero scale calibration
         while ((AdcSta(pADI_ADC1) &0x20) != 0x20);       // bit 5 set by adc when calibration is complete

         AdcGo(pADI_ADC1,ADCMDE_ADCMD_IDLE); //Set ADC channel in idle mode
         timer_sleep(1);

         calibFlag1 = false;


      }

}

