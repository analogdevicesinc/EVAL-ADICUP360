/*!
 *****************************************************************************
 * @file:    CN0398.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

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

/***************************** Library Include Files **************************/
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/***************************** Source Include Files ***************************/
#include "CN0398.h"
#include "ADuCM360.h"
#include "AD7124.h"
#include "Communication.h"
#include "Timer.h"
#include "DioLib.h"


/***************************** Class Variables ********************************/
/* Constants */
const float default_calibration_ph[2][2] = {{4, 0.169534}, {10,  -0.134135}};
const float default_offset_voltage = 0;
const uint16_t SENSOR_SETTLING_TIME = 400;
const float ph_temp_lut[NUMBER_OF_SOLUTIONS][NUMBER_OF_TEMPERATURE_ENTRIES] =
{
/* ACETATE    */ {4.667, 4.66, 4.655, 4.652, 4.651, 4.651, 4.65, 4.65, 4.65, 4.65, 4.65, 4.65, 4.65, 4.651, 4.651, 4.651, 4.652, 4.655, 4.656, 4.659, 4.666, 4.673, 4.683, 4.694, 4.706, 4.72, 4.736, 4.753, 4.772, 4.793, 4.815},
/* BORATE     */ {9.464, 9.395, 9.332, 9.276, 9.245, 9.235, 9.225, 9.216, 9.207, 9.197, 9.189, 9.18, 9.171, 9.163, 9.155, 9.147, 9.139, 9.102, 9.088, 9.068, 9.038, 9.01, 8.985, 8.962, 8.941, 8.921, 8.902, 8.884, 8.867, 8.85, 8.833},
/* CAOH2      */ {13.424, 13.207, 13.003, 12.81, 12.699, 12.663, 12.627, 12.592, 12.557, 12.522, 12.488, 12.454, 12.42, 12.387, 12.354, 12.322, 12.289, 12.133, 12.072, 11.984, 11.841, 11.705, 11.574, 11.449 },
/* CARBONATE  */ {10.317, 10.245, 10.179, 10.118, 10.084, 10.073, 10.062, 10.052, 10.042, 10.032, 10.022, 10.012, 10.002, 9.993, 9.984, 9.975, 9.966, 9.925, 9.91, 9.889, 9.857, 9.828},
/* CITRATE    */ {3.863, 3.84, 3.82, 3.803, 3.793, 3.791, 3.788, 3.785, 3.783, 3.78, 3.778, 3.776, 3.774, 3.772, 3.77, 3.768, 3.766, 3.759, 3.756, 3.754, 3.75, 3.749},
/* HCL        */ {1.082, 1.085, 1.087, 1.089, 1.09, 1.091, 1.091, 1.092, 1.092, 1.093, 1.093, 1.094, 1.094, 1.094, 1.095, 1.095, 1.096, 1.098, 1.099, 1.101, 1.103, 1.106, 1.108, 1.111, 1.113, 1.116, 1.119, 1.121, 1.124, 1.127, 1.13},
/* OXALATE    */ {1.666, 1.668, 1.67, 1.672, 1.674, 1.675, 1.675, 1.676, 1.677, 1.678, 1.678, 1.679, 1.68, 1.681, 1.681, 1.682, 1.683, 1.688, 1.69, 1.694, 1.7, 1.707, 1.715, 1.723, 1.732, 1.743, 1.754, 1.765, 1.778, 1.792, 1.806},
/* PHOSPHATE0 */ {6.984, 6.951, 6.923, 6.9, 6.888, 6.884, 6.881, 6.877, 6.874, 6.871, 6.868, 6.865, 6.862, 6.86, 6.857, 6.855, 6.853, 6.844, 6.841, 6.838, 6.834, 6.833, 6.833, 6.836, 6.84, 6.845, 6.852, 6.859, 6.867, 6.876, 6.886},
/* PHOSPHATE1 */ {7.118, 7.087, 7.059, 7.036, 7.024, 7.02, 7.016, 7.013, 7.009, 7.006, 7.003, 7, 6.997, 6.994, 6.992, 6.989, 6.987, 6.977, 6.974, 6.97, 6.965, 6.964, 6.965, 6.968, 6.974, 6.982, 6.992, 7.004, 7.018, 7.034, 7.052},
/* PHOSPHATE2 */ {7.534, 7.5, 7.472, 7.448, 7.436, 7.432, 7.429, 7.425, 7.422, 7.419, 7.416, 7.413, 7.41, 7.407, 7.405, 7.402, 7.4, 7.389, 7.386, 7.38, 7.373, 7.367},
/* PHTHALATE  */ {4, 3.998, 3.997, 3.998, 3.999, 4, 4.001, 4.001, 4.002, 4.003, 4.004, 4.005, 4.006, 4.007, 4.008, 4.009, 4.011, 4.018, 4.022, 4.027, 4.038, 4.05, 4.064, 4.08, 4.097, 4.116, 4.137, 4.159, 4.183, 4.208, 4.235},
/* TARTRATE   */ {3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.556, 3.555, 3.554, 3.553, 3.552, 3.549, 3.548, 3.547, 3.547, 3.549, 3.554, 3.56, 3.569, 3.58, 3.593, 3.61, 3.628, 3.65, 3.675},
/* TRIS       */ {8.471, 8.303, 8.142, 7.988, 7.899, 7.869, 7.84, 7.812, 7.783, 7.755, 7.727, 7.699, 7.671, 7.644, 7.617, 7.59, 7.563, 7.433, 7.382, 7.307, 7.186, 7.07},
/* PH4        */ {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 },
/* PH10       */ {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 },
};
const uint8_t ph_temperatures[NUMBER_OF_TEMPERATURE_ENTRIES] =
{
      0 ,      5 ,      10,      15,      18,      19,      20,      21,      22,      23,      24,      25,
      26,      27,      28,      29,      30,      35,      37,      40,      45,      50,      55,      60,
      65,      70,      75,      80,      85,      90,      95,
};
const char solutions[NUMBER_OF_SOLUTIONS][20] =
{
      "ACETATE",
       "BORATE",
       "CAOH2",
       "CARBONATE",
       "CITRATE",
       "HCL",
       "OXALATE",
       "PHOSPHATE0",
       "PHOSPHATE1",
       "PHOSPHATE2",
       "PHTHALATE",
       "TARTRATE",
       "TRIS",
       "PH4",
       "PH10"
};

/* Private */
float offset_voltage;
float calibration_ph[2][2];
uint8_t solution0, solution1, use_nernst = 1;

/* GLobal */
float temperature = 0, pH = 0, voltage[2] = {0,0}, moisture = 0;
int32_t adcValue[3];

void CN0398_Init(void)
{
   offset_voltage = default_offset_voltage;
   calibration_ph[0][0] = default_calibration_ph[0][0];
   calibration_ph[0][1] = default_calibration_ph[0][1];
   calibration_ph[1][0] = default_calibration_ph[1][0];
   calibration_ph[1][1] = default_calibration_ph[1][1];
   solution0 = 0;
   solution1 = 0;
   AD7124_Setup();
}

void CN0398_Setup(void)
{
   CN0398_Init();

   //Configure ADP7118 pin
   //DioCfgPin(ADP7118_PORT, ADP7118_PIN, 0);
   DioPulPin(ADP7118_PORT, ADP7118_PIN, 0);         /* Disable the internal pull up on AD7798 CS pin */
   DioOenPin(ADP7118_PORT, ADP7118_PIN, 1);         /* Set CS pin for AD7798 as output */
   DioClr(ADP7118_PORT, ADP7118_BIT);

   uint32_t setValue;
   enum ad7124_registers regNr;

   // Set Config_0 0x19
   regNr    = AD7124_Config_0;               //Select Config_0 register - RTD
   setValue = 0;//ad7124.ReadDeviceRegister(regNr);
   setValue |= AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
   setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
   setValue |= AD7124_CFG_REG_REF_BUFP;
   setValue |= AD7124_CFG_REG_REF_BUFM;
   setValue |= AD7124_CFG_REG_AIN_BUFP;
   setValue |= AD7124_CFG_REG_AINN_BUFM;
   setValue |= AD7124_CFG_REG_REF_SEL(1); //REFIN2(+)/REFIN2(−).
   setValue |= AD7124_CFG_REG_PGA(4);
   setValue &= 0xFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);   // Write data to ADC
   timer_sleep(5);
   //setValue = AD7124_ReadDeviceRegister(regNr);
   //printf("setValue = 0x%x\n", setValue);

   regNr = AD7124_Config_1;               //Select Config_1 register - pH
   setValue = 0;//ad7124.ReadDeviceRegister(regNr);
   setValue |= AD7124_CFG_REG_BIPOLAR;     //Select bipolar operation
   setValue |= AD7124_CFG_REG_BURNOUT(0);  //Burnout current source off
   setValue |= AD7124_CFG_REG_REF_BUFP;
   setValue |= AD7124_CFG_REG_REF_BUFM;
   setValue |= AD7124_CFG_REG_AIN_BUFP;
   setValue |= AD7124_CFG_REG_AINN_BUFM;
   setValue |= AD7124_CFG_REG_REF_SEL(0); //REFIN1(+)/REFIN1(-).
   setValue |= AD7124_CFG_REG_PGA(0); // gain1
   setValue &= 0xFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);   // Write data to ADC
   timer_sleep(5);
   //setValue = AD7124_ReadDeviceRegister(regNr);
   //printf("setValue = 0x%x\n", setValue);

   // Set Channel_0 register 0x09
   regNr = AD7124_Channel_0;  // RTD
   setValue = 0;
   setValue |= AD7124_CH_MAP_REG_SETUP(0);             // Select setup0
   setValue |= AD7124_CH_MAP_REG_AINP(9);         // Set AIN9 as positive input
   setValue |= AD7124_CH_MAP_REG_AINM(10);         // Set AIN10 as negative input
   setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE);  //Disable channel
   setValue &= 0xFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);   // Write data to ADC
   timer_sleep(5);
   //setValue = AD7124_ReadDeviceRegister(regNr);
   //printf("setValue = 0x%x\n", setValue);

   regNr = AD7124_Channel_1; // pH
   setValue = 0;
   setValue |= AD7124_CH_MAP_REG_SETUP(1);             // Select setup2
   setValue |= AD7124_CH_MAP_REG_AINP(6);         // Set AIN8 as positive input
   setValue |= AD7124_CH_MAP_REG_AINM(7);         // Set gnd as negative input
   setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE);  //Disable channel
   setValue &= 0xFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);   // Write data to ADC
   timer_sleep(5);
   //setValue = AD7124_ReadDeviceRegister(regNr);
   //printf("setValue = 0x%x\n", setValue);

   regNr = AD7124_Channel_2; // moisture
   setValue = 0;
   setValue |= AD7124_CH_MAP_REG_SETUP(1);             // Select setup2
   setValue |= AD7124_CH_MAP_REG_AINP(8);         // Set AIN8 as positive input
   setValue |= AD7124_CH_MAP_REG_AINM(19);         // Set gnd as negative input
   setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE);  //Disable channel
   setValue &= 0xFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);   // Write data to ADC
   timer_sleep(5);
   //setValue = AD7124_ReadDeviceRegister(regNr);
   //printf("setValue = 0x%x\n", setValue);

   // Set IO_Control_1 0x03
   regNr = AD7124_IOCon1;               //Select IO_Control_1 register
   //printf("IOCON1\n");
   setValue = 0;
   setValue |= AD7124_8_IO_CTRL1_REG_GPIO_CTRL2; // enable AIN3 as digital output
   setValue |= AD7124_8_IO_CTRL1_REG_GPIO_CTRL3; // enable AIN4 as digital output
   setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(11); // source ain11
   setValue |= AD7124_IO_CTRL1_REG_IOUT_CH1(12); // source ain12
   setValue |= AD7124_IO_CTRL1_REG_IOUT0(0x4);// set IOUT0 current to 500uA
   setValue |= AD7124_IO_CTRL1_REG_IOUT1(0x4);// set IOUT0 current to 500uA
   setValue &= 0xFFFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);// Write data to ADC
   timer_sleep(5);

   // Set IO_Control_2
   regNr = AD7124_IOCon2;               //Select IO_Control_2 register
   setValue = 0;
   setValue |= AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7; //enable bias voltage on AIN7
   setValue &= 0xFFFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);// Write data to ADC
   timer_sleep(5);

   // Set ADC_Control 0x01
   regNr = AD7124_ADC_Control;            //Select ADC_Control register
   setValue = AD7124_ReadDeviceRegister(regNr);
   setValue |= AD7124_ADC_CTRL_REG_DATA_STATUS; // set data status bit in order to check on which channel the conversion is
   setValue &= 0xFFC3; // remove prev mode bits
   setValue |= AD7124_ADC_CTRL_REG_MODE(1); //standby mode
   setValue &= 0xFFFF;
   AD7124_WriteDeviceRegister(regNr, setValue);    // Write data to ADC
   timer_sleep(5);
}

float CN0398_read_rtd(void)
{
   float temperature = 0, resistance = 0;

   int32_t data = 0;
   data = CN0398_read_channel(RTD_CHANNEL);

   adcValue[RTD_CHANNEL] = data;

   resistance = (((float)((data) - _2_23) * RREF)) / (TEMP_GAIN * _2_23);

   #ifdef USE_LINEAR_TEMP_EQ
      temperature = PT100_RESISTANCE_TO_TEMP(resistance);
   #else

      #define A (3.9083*pow(10,-3))
      #define B (-5.775*pow(10,-7))
         if (resistance < R0)
            temperature = ((resistance / R0) - 1) / 0.00385;
            //temperature = -242.02 + 2.228 * resistance + (2.5859 * pow(10, -3)) * pow(resistance, 2) - (48260.0 * pow(10, -6)) * pow(resistance, 3) - (2.8183 * pow(10, -3)) * pow(resistance, 4) + (1.5243 * pow(10, -10)) * pow(resistance, 5);
         else
            temperature = ((-A + sqrt((double)(pow(A, 2) - 4 * B * (1 - resistance / R0))) ) / (2 * B));
   #endif

   return temperature;
}

int32_t CN0398_read_channel(uint8_t ch)
{
   int32_t data;

   CN0398_enable_channel(ch);

   convFlag = 1;
   DioClr(CN0398_CS_PORT, CN0398_CS_BIT);
   timer_sleep(1);

   CN0398_start_single_conversion();

   if (AD7124_WaitForConvReady(10000) == -3)
   {
     printf("TIMEOUT\n");
   }

   AD7124_ReadData(&data);

   convFlag = 0;
   DioSet(CN0398_CS_PORT, CN0398_CS_BIT);
   timer_sleep(1);

   CN0398_disable_channel(ch);

   return data;
}

void CN0398_start_single_conversion(void)
{
   enum ad7124_registers regNr = AD7124_ADC_Control; //Select ADC_Control register
   AD7124_WriteDeviceRegister(regNr, 0x0584);    // Write data to ADC
   timer_sleep(1);
}

void CN0398_disable_channel(int channel)
{
    enum ad7124_registers regNr = (enum ad7124_registers)(AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = AD7124_ReadDeviceRegister(regNr);
    //printf("Ch %d: SetValue 0x%x\n", channel, setValue);
    //setValue &= (~(uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);  //Disable channel
    //setValue &= 0xFFFF;
    switch (channel)
    {
      case 0:
        setValue = 0x012a;
        break;
      case 1:
        setValue = 0x10c7;
        break;
      case 2:
        setValue = 0x1113;
        break;
    }
    AD7124_WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    timer_sleep(5);
}

void CN0398_enable_channel(int channel)
{
    enum ad7124_registers regNr = (enum ad7124_registers)(AD7124_Channel_0 + channel); //Select ADC_Control register
    uint32_t setValue = AD7124_ReadDeviceRegister(regNr);
    //printf("Ch %d: SetValue 0x%x\n", channel, setValue);
    //setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE;  //Enable channel0
    //setValue &= 0xFFFF;
    switch (channel)
    {
       case RTD_CHANNEL:
          setValue = 0x812a;
          break;
       case PH_CHANNEL:
          setValue = 0x90c7;
          break;
       case MOISTURE_CHANNEL:
          setValue = 0x9113;
          break;
    }
    AD7124_WriteDeviceRegister(regNr, setValue);    // Write data to ADC
    timer_sleep(5);
}

float CN0398_read_ph(float temperature)
{
   float ph = 0;

   #ifdef PH_SENSOR_PRESENT
      int32_t data = 0;
      float volt, m;
      CN0398_set_digital_output(P2, 1);

      data = CN0398_read_channel(PH_CHANNEL);
      adcValue[PH_CHANNEL] = data;

      voltage[PH_CHANNEL - 1] = CN0398_data_to_voltage_bipolar(data, 1, 3.3);
      volt = voltage[PH_CHANNEL - 1];

      if(use_nernst)
      {
         ph  = PH_ISO -((volt - ZERO_POINT_TOLERANCE) / ((2.303 * AVOGADRO * (temperature + KELVIN_OFFSET)) / FARADAY_CONSTANT) );
      }
      else
      {
         m =  (calibration_ph[1][0] - calibration_ph[0][0]) / (calibration_ph[1][1] - calibration_ph[0][1]);
         ph = m * (volt - calibration_ph[1][1] + offset_voltage) + calibration_ph[1][0];
      }

      CN0398_set_digital_output(P2, 0);
   #endif

   return ph;
}

void CN0398_set_digital_output(uint8_t p, uint8_t state)
{
   enum ad7124_registers regNr = AD7124_IOCon1; //Select ADC_Control register

   uint32_t setValue = AD7124_ReadDeviceRegister(regNr);
   if(state)
      setValue |= ((AD7124_8_IO_CTRL1_REG_GPIO_DAT1) << p);
   else
      setValue &= (~(AD7124_8_IO_CTRL1_REG_GPIO_DAT1 << p));

   AD7124_WriteDeviceRegister(regNr, setValue);    // Write data to ADC
   timer_sleep(1);
}

float CN0398_read_moisture(void)
{
   float moisture = 0;

   #ifdef MOISTURE_SENSOR_PRESENT
      int32_t data;
      float volt;

      DioSet(ADP7118_PORT, ADP7118_BIT);
      CN0398_set_digital_output(P3, 1);
      timer_sleep(SENSOR_SETTLING_TIME);

      adcValue[MOISTURE_CHANNEL]= CN0398_read_channel(MOISTURE_CHANNEL);
      data = adcValue[MOISTURE_CHANNEL];
      DioClr(ADP7118_PORT, ADP7118_BIT);

      voltage[MOISTURE_CHANNEL - 1] = CN0398_data_to_voltage_bipolar(data, 1, 3.3);
      volt = voltage[MOISTURE_CHANNEL - 1];

      #ifdef USE_MANUFACTURER_MOISTURE_EQ
         if (volt <= 1.1)
            {
           moisture = 10 * volt - 1;
         }
         else if (volt > 1.1 && volt <= 1.3)
         {
           moisture = 25 * volt - 17.5;
         }
         else if (volt > 1.3 && volt <= 1.82) {
           moisture = 48.08 * volt - 47.5;
         } else if (volt > 1.82) {
           moisture = 26.32 * volt - 7.89;
         }
      #else
         moisture = -1.18467 + 21.5371 * volt - 110.996 * (pow(volt, 2)) + 397.025 * (pow(volt, 3)) - 666.986 * (pow(volt, 4)) + 569.236 * (pow(volt, 5)) - 246.005 * (pow(volt, 6)) + 49.4867 * (pow(volt, 7)) - 3.37077 * (pow(volt, 8));
      #endif
      if (moisture > 100) moisture = 100;
      if (moisture < 0 ) moisture = 0;
      CN0398_set_digital_output(P3, 0);
   #endif
   return moisture;
}

float CN0398_data_to_voltage_bipolar(uint32_t data, uint8_t gain, float VREF)
{
   data = data & 0xFFFFFF;
   return ((float)(data / (float)(0xFFFFFF / 2)) - 1) * (VREF / gain);
}

void CN0398_calibrate(void)
{
   char response;

   uart_cmd = UART_FALSE;
   uart_read_ch = 0;
   printf("Calibrate CN0398:\n");
   printf("Do you want to perform pH calibration [Y/N]?\n");
   scanf("%c",&response);
   uart_cmd = UART_FALSE;
   printf("\n");
   if (response == 'y' || response == 'Y')
   {
      CN0398_calibrate_ph();
   }
   else
   {
      use_nernst = 1;
      printf("Do you want to load default calibration?[y/N]. If not[N], the Nernst equation will be used.\n");
      scanf("%c",&response);
      uart_cmd = UART_FALSE;
      if (response == 'y' || response == 'Y')
      {
         use_nernst = 0;
      }
   }
   printf("CN0398 System Calibration Complete\n");
}

void CN0398_calibrate_ph(void)
{
   char response;
   uint8_t response_ok;
   float temperature;

   printf("Calibrating pH:\n");

   CN0398_read_rtd();
   CN0398_read_rtd();

   printf("Do you want to calibrate offset voltage [Y/N]?\n");

   scanf("%c",&response);
   uart_cmd = UART_FALSE;
   printf("\n");

   if (response == 'y' || response == 'Y')
   {
      timer_sleep(5);
      uart_cmd = UART_FALSE;
      timer_sleep(5);
      printf("Calibration step 0. Short the pH probe and press <ENTER> to calibrate.\r\n");
      while(uart_cmd == UART_FALSE);
      uart_cmd = UART_FALSE;
      timer_sleep(1);
      CN0398_calibrate_ph_offset();
   }
   CN0398_print_calibration_solutions();

   response_ok = 0;
   while (response_ok == 0)
   {
      printf("Input calibration solution used for first step [0-9][a-e]:\n");
      scanf("%c",&response);
      uart_cmd = UART_FALSE;
      if (response >= '0' && response <= '9')
      {
         response_ok = 1;
         solution0 = response - '0';
      }
      else if(response>='A' && response <= 'E')
      {
         response_ok = 1;
         solution0 = response - 'A' + 10;
      }
      else if(response>='a' && response <= 'e')
      {
         response_ok = 1;
         solution0 = response - 'a' + 10;
      }
      else
      {
         response_ok = 0;
      }
   }
   printf("%s solution selected. Solution pH at 25°C = %.3f \n", solutions[solution0], ph_temp_lut[solution0][11]);
   printf("\n");

   temperature = CN0398_read_rtd();
   timer_sleep(5);
   uart_cmd = UART_FALSE;
   timer_sleep(5);
   printf("Calibration step 1. Place pH probe in first calibration solution and press <ENTER> to start calibration.\r\n");
   while(uart_cmd == UART_FALSE);
   uart_cmd = UART_FALSE;
   CN0398_calibrate_ph_pt0(temperature);

   response_ok = false;
   while (response_ok == false)
   {
      printf("Input calibration solution used for second step [0-9][a-e]:\n");
      scanf("%c",&response);
      uart_cmd = UART_FALSE;
      if(response >= '0' && response <= '9')
      {
         response_ok = true;
         solution1 = response - '0';
      }
      else if(response>='A' && response <= 'E')
      {
         response_ok = true;
         solution1 = response - 'A' + 10;
      }
      else if(response>='a' && response <= 'e')
      {
         response_ok = true;
         solution1 = response - 'a' + 10;
      }
      else
      {
         response_ok = false;
      }
   }
   printf("%s solution selected. Solution pH at 25°C = %.3f \n", solutions[solution1], ph_temp_lut[solution1][11]);
   printf("\n");
   timer_sleep(5);
   uart_cmd = UART_FALSE;
   timer_sleep(5);
   printf("Calibration step 2. Place pH probe in second calibration solution and press <ENTER> to start calibration.\r\n");
   while(uart_cmd == UART_FALSE);
   uart_cmd = UART_FALSE;
   CN0398_calibrate_ph_pt1(temperature);
   printf("\n");
   printf("\n");
}

void CN0398_calibrate_ph_offset(void)
{
   int32_t data = 0;
   float volt;
   CN0398_set_digital_output(P2, 1);

   data = CN0398_read_channel(PH_CHANNEL);
   //printf("Data: 0x%x\n", data);
   volt = CN0398_data_to_voltage_bipolar(data, 1, 3.3);

   offset_voltage = volt;
   printf("Offset voltage is %fV\n", volt);
   printf("\n");

   CN0398_set_digital_output(P2, 0);
}

void CN0398_print_calibration_solutions(void)
{
   int i;

   printf("Calibration solutions available for two point calibration:\n");
   for (i = 0; i < NUMBER_OF_SOLUTIONS; i++)
   {
         printf("%x. %s\n", i, solutions[i]);
   }
   printf("\n");
}

void CN0398_calibrate_ph_pt0(float temperature)
{
   int32_t data;
   float volt;

   CN0398_set_digital_output(P2, 1);
   data = CN0398_read_channel(PH_CHANNEL);
   volt = CN0398_data_to_voltage_bipolar(data, 1, 3.3);

   if (temperature < 0)
   {
     calibration_ph[0][0] = ph_temp_lut[solution0][0];
   }
   else
   {
      for (uint8_t i = 1; i < NUMBER_OF_TEMPERATURE_ENTRIES; i++)
      {
         if(temperature > ph_temperatures[i - 1] && temperature <= ph_temperatures[i]) {
            calibration_ph[0][0] = ph_temp_lut[solution0][i];
            break;
         }
      }
   }

   calibration_ph[0][1] =  volt;
   CN0398_set_digital_output(P2, 0);

   printf("Calibration solution1 pH = %.3f with sensor voltage of %fV\n", calibration_ph[0][0], volt);
   printf("\n");
}

void CN0398_calibrate_ph_pt1(float temperature)
{
   int32_t data;
   float volt;

   CN0398_set_digital_output(P2, 1);
   data = CN0398_read_channel(PH_CHANNEL);
   volt = CN0398_data_to_voltage_bipolar(data, 1, 3.3);

   if (temperature < 0)
   {
      calibration_ph[1][0] = ph_temp_lut[solution1][0];
   }
   else
   {
      for (uint8_t i = 1; i < NUMBER_OF_TEMPERATURE_ENTRIES; i++) {
         if (temperature > ph_temperatures[i - 1] && temperature <= ph_temperatures[i])
         {
            calibration_ph[1][0] = ph_temp_lut[solution1][i];
            break;
         }
      }
   }

   calibration_ph[1][1] =  volt;
   CN0398_set_digital_output(P2, 0);

   printf("Calibration solution2 pH = %.3f with sensor voltage of %fV\n", calibration_ph[1][0], volt);
   printf("\n");
}

void CN0398_set_data(void)
{
   temperature = CN0398_read_rtd();
   pH = CN0398_read_ph(temperature);
   moisture = CN0398_read_moisture();
}

void CN0398_display_data(void)
{
   printf("Temperature = %f°C\n",temperature);
   printf("pH = %f\n", pH);
   printf("Moisture = %f%c\n", moisture, 37);
   printf("\n");
   printf("\n");
}
