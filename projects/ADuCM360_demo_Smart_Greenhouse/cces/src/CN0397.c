/***************************** Library Include Files **************************/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/***************************** Source Include Files ***************************/
#include "ADuCM360.h"
#include "Timer.h"
#include "Communication.h"
#include "DioLib.h"
#include "UrtLib.h"
#include "AD7798.h"
#include "CN0397.h"

/***************************** Class Variables ********************************/
/* Constants */
const uint8_t Gain[8] = { 1, 2, 4, 8, 16, 32, 64, 128};
const uint8_t ColorPrint[3] = { 31, 32, 34 };
const uint8_t Channels[CHANNELS] = { 1, 0, 2};
const float Lux_LSB[3] = {2.122, 2.124, 2.113};
const float Optimal_Levels[3] = {26909.0, 8880.0, 26909.0};
static const char *colour[] = {
   [0] = "RED",
   [1] = "GREEN",
   [2] = "BLUE",
};

/* Global */
uint8_t statusReg, idReg, ioReg, gainAdc;
uint16_t cn0397_adcValue[3], modeReg, configReg, offsetReg, fullscaleReg, dataReg;
int barLine[3];
float voltageValue[3] = {0,0,0}, intensityValue[3] = {0,0,0}, lightConcentration[3] = {0,0,0};


void CN0397_Init(void)
{
   AD7798_Reset();

   if (AD7798_Init())
   {
      AD7798_SetCodingMode(AD7798_UNIPOLAR);
      AD7798_SetMode(AD7798_MODE_SINGLE);
      AD7798_SetGain(ADC_GAIN);
      AD7798_SetFilter(ADC_SPS);
      AD7798_SetReference(AD7798_REFDET_ENA);
   }

   gainAdc = Gain[ADC_GAIN];
}

void CN0397_StartCal(void)
{
   uint8_t channel;

   printf("Calibrate CN0397:\n");
   printf("\n");

   for (channel = 0; channel < CHANNELS; channel++)
   {
      uart_cmd = UART_FALSE;
      printf("Calibrate %s channel: be sure that %s photodiode is covered and press <ENTER>.\n", colour[channel], colour[channel]);
      while (uart_cmd == UART_FALSE);
      printf("Calibrating...\n");
      CN0397_Calibration(Channels[channel]);
      printf("Channel is calibrated!\n");
      printf("\n");
   }
   uart_cmd = UART_FALSE;
   printf("CN0397 system calibration complete!\n");
   printf("\n");
}

void CN0397_ReadADCData(uint8_t adcChannel, uint16_t *adcData)
{
   uint8_t channel;

   channel = 0x80 | adcChannel;

   convFlag = 1;

   DioClr(CN0397_CS_PORT, CN0397_CS_BIT);

   AD7798_SetRegisterValue(AD7798_REG_MODE, 0x200A, 2);

   while ((AD7798_GetRegisterValue( AD7798_REG_STAT,1) & channel) != channel);

   timer_sleep(200);

   *adcData = AD7798_GetRegisterValue(AD7798_REG_DATA,2);

   DioSet(CN0397_CS_PORT, CN0397_CS_BIT);

   convFlag = 0;
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

   while ((AD7798_GetRegisterValue(AD7798_REG_STAT,1) & channel) != channel);

   printf("Idling...\n");

   while ((AD7798_GetRegisterValue(AD7798_REG_MODE, 2) & AD7798_MODE_SEL(0x7)) != AD7798_MODE_SEL(AD7798_MODE_IDLE));
}

void CN0397_ConvertToVoltage(uint16_t temp_adcValue, float *voltage)
{
   *voltage = (float)(temp_adcValue * V_REF)/(float)(_2_16 * gainAdc);
}

void CN0397_CalcLightIntensity(uint8_t channel, uint16_t temp_adcValue, float *intensity)
{
   *intensity = temp_adcValue * Lux_LSB[channel];
}

void CN0397_CalcLightConcentration(uint8_t channel, float intensity, float *conc)
{
   *conc = (intensity *100)/Optimal_Levels[channel];
}

void CN0397_SetBar(float conc, int *line)
{
   float concLimit = 5.0;
   int i = 0, j;
   *line = 0;

   if (conc > 0.0)
   {
      i = 1;
      *line = i;
   }

   for (j = 0; j< 20; j++)
   {
      if (conc >= concLimit)
      {
         *line = i+1;
      }
      concLimit +=5.0;
      i +=1;
   }
}

void CN0397_SetAppData(void)
{
   uint8_t channel, rgbChannel;

   for (channel = 0; channel < CHANNELS; channel++)
   {
      rgbChannel = Channels[channel];

      AD7798_SetChannel(channel);

      CN0397_ReadADCData(channel, &cn0397_adcValue[rgbChannel]);
      CN0397_ConvertToVoltage(cn0397_adcValue[rgbChannel], &voltageValue[rgbChannel]);
      CN0397_CalcLightIntensity(rgbChannel, cn0397_adcValue[rgbChannel], &intensityValue[rgbChannel]);
      CN0397_CalcLightConcentration(rgbChannel, intensityValue[rgbChannel], &lightConcentration[rgbChannel]);
      CN0397_SetBar(lightConcentration[rgbChannel], &barLine[rgbChannel]);
   }
}

void CN0397_DisplayData(void)
{

   uint8_t channel, i;

   for (channel = 0; channel < CHANNELS; channel++)
   {
      printf("\t%s channel:[", colour[channel]);
      for (i = 0; i < barLine[channel] ; i++)
      {
         printf("\033[2;%dm%c\033[0m", ColorPrint[channel], 219);
      }
      for (i = 0; i < (21 - barLine[channel]) ; i++)
      {
            printf(" ");
      }
      printf("]");
      printf("\t");
   }

   printf("\n");
   printf("\t");

   for (channel = 0; channel < CHANNELS; channel++)
   {
      printf("\tLight Intensity = %.2f lux\t\t", intensityValue[channel]);
   }

   printf("\n");
   printf("\t");

   for (channel = 0; channel < CHANNELS; channel++)
   {
      printf("\tLight Concentration = %.2f%c\t\t", lightConcentration[channel], 37);
   }

   printf("\n");

   for (channel = 0; channel < 3; channel++)
   {
         printf("\n");
   }
}
