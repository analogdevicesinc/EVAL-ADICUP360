#ifndef CN0397_H_
   #define CN0397_H_

   #define ADC_GAIN           AD7798_GAIN_1
   #define ADC_SPS            0x05
   #define USE_CALIBRATION    YES

   #define CHANNELS           3

   #define V_REF              3150.0
   #define _2_16              65535.0

   extern void CN0397_Init(void);
   extern void CN0397_StartCal(void);
   extern void CN0397_ReadADCData(uint8_t adcChannel, uint16_t *adcData);
   extern void CN0397_Calibration(uint8_t channel);
   extern void CN0397_ConvertToVoltage(uint16_t adcValue, float *voltage);
   extern void CN0397_CalcLightIntensity(uint8_t channel, uint16_t adcValue, float *intensity);
   extern void CN0397_CalcLightConcentration(uint8_t channel, float intensity, float *conc);
   extern void CN0397_SetBar(float conc, int *line);
   extern void CN0397_SetAppData(void);
   extern void CN0397_DisplayData(void);

   extern uint16_t cn0397_adcValue[3];
   extern const uint8_t Channels[];
   extern const float Lux_LSB[3];


#endif
