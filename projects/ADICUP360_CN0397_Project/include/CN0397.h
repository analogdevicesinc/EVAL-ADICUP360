/**
******************************************************************************
*   @file     CN0397.h
*   @brief    Header file for communication part
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
#ifndef CN0397_H_
#define CN0397_H_

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "AD7798.h"

bool _enter;   // enter key

#define REGISTERS_VALUES     3
#define CONVERSION_DATA      4

#define YES       1
#define NO        0

#define CHANNELS  3

#define V_REF                3150.0    // [mV]
#define _2_16                65535.0   // 2^16

extern void CN0397_Init(void);
extern void CN0397_DisplayData(void);
extern void CN0397_ReadADCData(uint8_t adcChannel, uint16_t *adcData);
extern void CN0397_ConvertToVoltage(uint16_t adcValue, float *voltage);
extern void CN0397_CalcLightIntensity(uint8_t channel, uint16_t adcValue, float *intensity);
extern void CN0397_CalcLightConcentration(uint8_t channel, float intensity, float *conc);
extern void CN0397_SetAppData(void);
extern void CN0397_Calibration(uint8_t channel);
extern void CN0397_SetBar(float conc, int *line);

/* Available settings:
 *  AD7798_GAIN_1, AD7798_GAIN_2,
 *  AD7798_GAIN_4, AD7798_GAIN_8,
 *  AD7798_GAIN_16, AD7798_GAIN_32,
 *  AD7798_GAIN_64, AD7798_GAIN_128
 */
#define ADC_GAIN      AD7798_GAIN_1
/* Available settings:
 * Check available value from datasheet
 */
#define ADC_SPS        0x05  //50SPS


/* Available settings:
 * How often to display output values on terminal -> msec
 */
#define DISPLAY_REFRESH        500   //[msec]

#define USE_CALIBRATION   NO    // Set YES if you want to use system zero-scale calibration before reading the system data; otherwise set NO.

#endif /* CN0397_H_ */

