/*!
 *****************************************************************************
 * @file:    CN0396.h
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

#ifndef CN0396_H_
#define CN0396_H_

#include <stdio.h>
#include <string.h>
#include "AD7798.h"


#define REGISTERS_VALUES     3
#define CONVERSION_DATA      4

#define V_REF                1.2   // [V]
#define _2_16                65536.0   // 2^16
#define _2_15                32768.0   // 2^15

#define COMPENSATION_TABLE_SIZE 9

    typedef enum {
        CO_SENSOR,
        H2S_SENSOR
    } sensor_type_t;

  typedef struct {
        int8_t temp;
        float CO_percent;
        float H2S_percent;
    } ppm_compensation_t;


extern void CN0396_Init(void);
extern void CN0396_DisplayData(void);
extern void CN0396_ConvertToVoltage(uint16_t adcValue, float *voltage);
extern void CN0396_SetAppData(void);
extern float CN0396_GetFeedbackResistor(float sensitivity, float range);
extern float CN0396_CompensatePPM(float result, float temp, sensor_type_t sensor);
extern void CN0396_ConfigureFeedbackResistors(float resistance1, float resistance2);

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

#define MAX_CO_SENS  (100 * pow(10, -9))
#define CO_SENS      (75 * pow(10, -9))    /* Sensitivity nA/ppm CO 50 to 100 */
#define CO_RANGE     1000 /* Range ppm CO limit of performance warranty 1,000 */

#define MAX_H2S_SENS (1000 * pow(10, -9))
#define H2S_SENS     (800 * pow(10, -9)) /* Sensitivity nA/ppm  H2S 450 to 900 */
#define H2S_RANGE    200  /* Range ppm H2S limit of performance warranty 200 */


#endif /* CN0396_H_ */

