/**
******************************************************************************
*   @file    CN0394.h
*   @brief    Header file for CN0394
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
*******************************************************************************
**/

#ifndef CN0394_H_
#define CN0394_H_

#include <stdio.h>
#include <string.h>
#include <stdbool.h>


#define YES    1
#define NO     0

typedef enum
{
   CHANNEL_1 = 0,
   CHANNEL_2,
   CHANNEL_3,
   CHANNEL_4
} cn0394_channel;


extern int32_t adc0[4], adc1[4];
extern bool calibFlag0, calibFlag1;
extern uint8_t write0, write1;

extern void CN0394_DisplayData(void);
extern void CN0394_SetData(void);
extern void CN0394_Init(void);
extern float CN0394_CalcVoltage(uint8_t channel, int32_t adcValue);
extern void CN0394_ChannelConfig(uint8_t channel, uint8_t adcInput);
extern float CN0394_CalcCurrent(float voltage);
extern  float CN0394_CalcRrtd(int32_t adc);
extern void CN0394_StartConversion(uint8_t channel);
extern void CN0394_Calc_RTDTemperature(cn0394_channel ch, float *temp);
extern void CN0394_Calc_ThermocoupleTemperature(cn0394_channel ch, float cjTemp, float *buffer);
extern void CN0394_Calibration(uint8_t channel);


#define RTD_CHANNEL    0
#define TH_CHANNEL     1

#define R5             1600.0 //ohmi

#define I_EXC          0.620   // mA

#define VREF0          (R5*I_EXC)  //mV
#define VREF1          1200.0  //mV
#define _2_28          268435456.0

#define POLY_CALC(retVal, inVal, coeff_array) \
{ \
    float expVal = 1.0f; \
    const float* coeff = coeff_array; \
    retVal = 0.0f; \
    while(*coeff != 1.0f)\
    { \
        retVal += *coeff * expVal; \
        expVal *= inVal; \
        coeff++; \
    }\
}

/* Enable/disable calibration for each channel */
#define USE_RTD_CALIBRATION   YES              //YES or NO
#define USE_TH_CALIBRATION    YES              //YES or NO

#define DISPLAY_REFRESH   1000       //ms


#endif /* CN0394_H_ */

