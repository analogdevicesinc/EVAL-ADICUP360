/**
*   @file     CN0396.c
*   @brief    Source file for CN0396
*   @author   Analog Devices Inc.
*
********************************************************************************
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
#include <CN0396.h>
#include "Communication.h"

#include "ADuCM360.h"
#include "DioLib.h"
#include "SpiLib.h"
#include "UrtLib.h"

#include "AD7798.h"
#include "ADT7310.h"
#include "AD5270.h"

#include "Timer.h"

#include <stdio.h>
#include <math.h>

uint8_t statusReg, idReg, ioReg, gainAdc;
uint16_t modeReg, configReg, offsetReg, fullscaleReg, dataReg;

float Temp;
uint16_t data0, data1;
float volt0, volt1;
float result0[2], result1[2];

float resistance0, resistance1;

const uint8_t Gain[8] = { 1, 2, 4, 8, 16, 32, 64, 128};

const ppm_compensation_t ppm_compensation[COMPENSATION_TABLE_SIZE] = {
    { -30  , 29.9  , 82.3 },
    { -20  , 38.8  , 84.6 },
    { -10  , 53.7  , 88.6 },
    {0     , 69.6  , 92.2 },
    {10    , 84.9  , 96.2 },
    {20    , 100.0 , 100.0},
    {30    , 112.7 , 103.1},
    {40    , 123.7 , 105.6},
    {50    , 133.1 , 107.4},
};


/******************************************************************************
 * @brief Display measured and calculation data (on UART).
 *
 * @param None.
 *
 * @return None.
*******************************************************************************/
void CN0396_DisplayData(void)
{
  printf("Temperature = %f°C\n", Temp);
  printf("\n");
  printf("ADC channel1 (CO) = %u (%#06x)\n", data0, data0);
  printf("Voltage channel1 (CO) = %f V\n", volt0);
  printf("CO concentration = %f ppm\n", result0[0]);
  printf("CO concentration (interpolation) = %f ppm\n", result0[1]);
  printf("\n");
  printf("ADC channel2 (H2S) = %u (%#06x)\n", data1, data1);
  printf("Voltage channel2 (H2S) = %f V\n", volt1);
  printf("H2S concentration = %f ppm\n", result1[0]);
  printf("H2S concentration (interpolation) = %f ppm\n", result1[1]);
  printf("\n");
  printf("\n");
  printf("\n");

}

/******************************************************************************
 * @brief Convert ADC code into voltage value.
 *
 * @param adcValue - ADC code to convert.
 * @param *voltage - pointer where to store voltage value.
 *
 * @return None.
*******************************************************************************/
void CN0396_ConvertToVoltage(uint16_t adcValue, float *voltage)
{

   *voltage = ((float)(adcValue/_2_15) - 1.0)*(V_REF/gainAdc);

}

/******************************************************************************
 * @brief Initialization part.
 *
 * @param None.
 *
 * @return None.
*******************************************************************************/
void CN0396_Init(void)
{
   convFlag = 0;
   daisyCh = 0;

   SPI_Init();

   UART_Init(B115200, COMLCR_WLS_8BITS);

   resistance0 = CN0396_GetFeedbackResistor(MAX_CO_SENS,  CO_RANGE );
   resistance1 = CN0396_GetFeedbackResistor(MAX_H2S_SENS, H2S_RANGE);

   CN0396_ConfigureFeedbackResistors(resistance0, resistance1);


   ADT7310_Reset();

   ADT7310_WriteReg(ADT7310_CONFIG, 0x90, 1); // 16-bit ADC resolution and comparator mode selected

   AD7798_Reset();
   if(AD7798_Init()){

         AD7798_SetCodingMode(AD7798_BIPOLAR);
         AD7798_SetMode(AD7798_MODE_SINGLE);
         AD7798_SetGain(ADC_GAIN);
         AD7798_SetFilter(ADC_SPS);
         AD7798_SetReference(AD7798_REFDET_ENA);

   }

   gainAdc = Gain[ADC_GAIN];
}

/******************************************************************************
 * @brief Set all application data.
 *
 * @param None.
 *
 * @return None.
*******************************************************************************/
void CN0396_SetAppData(void)
{

   Temp = ADT7310_ReadTemp();

    AD7798_SetChannel(AD7798_CH1);
    AD7798_ReadData(AD7798_CH1, &data0);
    CN0396_ConvertToVoltage(data0, &volt0);
    result0[0] = (float)((volt0/resistance0)/CO_SENS);

    AD7798_SetChannel(AD7798_CH2);
    AD7798_ReadData(AD7798_CH2, &data1);
    CN0396_ConvertToVoltage(data1, &volt1);
    result1[0] = (float)((volt1/resistance1)/H2S_SENS);

    result0[1] = CN0396_CompensatePPM(result0[0], Temp, CO_SENSOR);
    result1[1] = CN0396_CompensatePPM(result1[0], Temp, H2S_SENSOR);

}

/******************************************************************************
 * @brief Get feedback resistor value.
 *
 * @param sensitivity - sensitivity value based on sensor type.
 * @param range - range used based on sensor type.
 *
 * @return float - resistor value.
*******************************************************************************/
float CN0396_GetFeedbackResistor(float sensitivity, float range)
{
    return (V_REF / (sensitivity * range));
}

/******************************************************************************
 * @brief Configure feedback resistor value.
 *
 * @param resistance1 - R1 value.
 * @param resistance2 - R2 value.
 *
 * @return None.
*******************************************************************************/
void CN0396_ConfigureFeedbackResistors(float resistance1, float resistance2)
{
    uint16_t R1 = AD5270_CalcRDAC(resistance1);
    uint16_t R2 = AD5270_CalcRDAC(resistance2);

    SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
           SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_LOW |
           SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);

    daisyCh = 1;

    DioClr(AD5270_CS_PORT, AD5270_CS_PIN);
    AD5270_WriteReg(WRITE_CTRL_REG, RDAC_WRITE_PROTECT);
    AD5270_WriteReg(WRITE_CTRL_REG, RDAC_WRITE_PROTECT);
    DioSet(AD5270_CS_PORT, AD5270_CS_PIN);


    DioClr(AD5270_CS_PORT, AD5270_CS_PIN);
    AD5270_WriteReg(WRITE_RDAC, R2);
    AD5270_WriteReg(WRITE_RDAC, R1);
    DioSet(AD5270_CS_PORT, AD5270_CS_PIN);


    DioClr(AD5270_CS_PORT, AD5270_CS_PIN);
    AD5270_WriteReg(WRITE_CTRL_REG, 0);
    AD5270_WriteReg(WRITE_CTRL_REG, 0);
    DioSet(AD5270_CS_PORT, AD5270_CS_PIN);

    AD5270_Set_SDO_HiZ();

    daisyCh = 0;

    SpiCfg(pADI_SPI1, SPICON_MOD_TX4RX4, SPICON_MASEN_EN, SPICON_CON_EN | SPICON_SOEN_EN |
           SPICON_RXOF_EN | SPICON_ZEN_EN | SPICON_TIM_TXWR | SPICON_CPOL_HIGH |
           SPICON_CPHA_SAMPLETRAILING | SPICON_ENABLE_EN);
}

/******************************************************************************
 * @brief Calculate compensated PPM value.
 *
 * @param resultIn - input PPM value.
 * @param temp - temperature value.
 * @param sensor - sensor type.
 *
 * @return float - compensated PPM value.
*******************************************************************************/
float CN0396_CompensatePPM(float resultIn, float temp, sensor_type_t sensor)
{
   float compensation_coef;
   float resultOut = 6000.0;

    for(uint8_t i = 1; i < COMPENSATION_TABLE_SIZE; i++) {

        if(temp < ppm_compensation[i].temp && temp > ppm_compensation[i - 1].temp) {

            if(sensor == H2S_SENSOR) {
                compensation_coef = (((temp - (ppm_compensation[i - 1].temp )) * (ppm_compensation[i].H2S_percent - ppm_compensation[i - 1].H2S_percent)) / (ppm_compensation[i].temp  - ppm_compensation[i - 1].temp)) + ppm_compensation[i - 1].H2S_percent;
            } else {
                compensation_coef = (((temp - (ppm_compensation[i - 1].temp )) * (ppm_compensation[i].CO_percent - ppm_compensation[i - 1].CO_percent)) / (ppm_compensation[i].temp  - ppm_compensation[i - 1].temp)) + ppm_compensation[i - 1].CO_percent;
            }

            resultOut = (resultIn * compensation_coef) / 100.0;

        }
    }

    return resultOut;
}

