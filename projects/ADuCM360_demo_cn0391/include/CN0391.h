/****************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
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
********************************************************************************/
#ifndef _CN0391_H_
#define _CN0391_H_
#include "AD7124.h"

class CN0391
{
private:
public:
	CN0391();

	typedef enum
	{
		CHANNEL_P1 = 0,
		CHANNEL_P2,
		CHANNEL_P3,
		CHANNEL_P4
	}channel_t;

   typedef enum
   {
      ERR_UNDER_RANGE = 1,
      ERR_OVER_RANGE,
      NO_ERR

   }error_code;


	int32_t read_channel(int ch);

	float data_to_voltage(int32_t data, uint8_t channel);
	float data_to_resistance(int32_t data);

	void calc_rtd_temperature(channel_t ch, float *temp);
	void calc_th_temperature(channel_t ch, float cjTemp, float *buffer);

	void enable_channel(int channel);
	void disable_channel(int channel);
	void enable_current_source(int current_source_channel);
	void start_single_conversion();

	void reset();
	void setup();
	void init();

   void calibration(uint8_t channel);
   void read_reg(void);
   void set_calibration_mode(uint16_t mode);
   void set_power_mode(int mode);

	void set_data(void);
	void display_data(void);

	AD7124 ADC;


};

#define YES    1
#define NO     0


#define R5      1600.0  //ohmi
#define I_EXT   0.75    //mA

#define VREF_EXT    (R5*I_EXT)  //mV
#define VREF_INT     2500.0     //mV

#define _2_23     8388608.0

#define RTD_CHANNEL    0

#define TH_CHANNEL     1

#define GAIN_RTD       1  //GAIN1
#define GAIN_TH        32  //GAIN32


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


#define USE_RTD_CALIBRATION  YES  // Set YES to enable calibration on RTD channel, otherwise set to NO
#define USE_TH_CALIBRATION   YES   // Set YES to enable calibration on TC channel, otherwise set to NO

#define DISPLAY_REFRESH     (1000)   //ms

#define TC_OFFSET_VOLTAGE    0.00    // mV compensation for system offset


#endif
