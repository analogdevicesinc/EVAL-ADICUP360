/******************************************************************************
*   @file     CN0395.h
*   @brief    Header file for CN0395
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2017(c) Analog Devices, Inc.
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

#ifndef CN0395_H_
#define CN0395_H_

/********************************* Global data ********************************/
#define ALPHA      0.003074

/****************************** Internal types *********************************/
typedef struct {
   float       fAmbientHeaterTemp;  // T_A
   float       fAmbientHeaterHum;   // HUM
   float       fAmbientHeaterRes;   // RH_A
   float       fHeaterVoltage;      // VH
   float       fHeaterCurrent;      // IH
   float       fHeaterRes;          // RH
   float       fHeaterTemp;         // TH
   float       fHeaterPower;        // PH
   float       fSensorRes;          // RS
   float       fSensorResCleanAir;  // Ro
   float       fSensorVoltage;      // VS
   float       fPPM;                // PPM
   uint32_t    ui32GainResistance;  // RG
   uint16_t    ui16LastAdcDataRead;
   uint8_t     OpMode;
   float       K1;                  // Gain correction factor
}sMeasurementVariables;

typedef enum{
   VOLTAGE,
   RESISTANCE
}enCN0395ErrCorrection;

typedef  void (*cmdFunc)(uint8_t *, sMeasurementVariables *);

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Calls the init functions for SPI, I2C and UART */
void CN0395_Init(void);
/* Command line process function */
void CN0395_CmdProcess(sMeasurementVariables *sMeasVar);
/* Find available commands */
cmdFunc CN0395_FindCommand(char *cmd);
/* Command Prompt function */
int CN0395_CmdPrompt(void);
/* Function which performs the power up routine */
void CN0395_PowerOn(sMeasurementVariables *sMeas);
/* Routine for measuring sensor resistance RS */
float CN0395_MeasureSensorResistance(sMeasurementVariables *sMeasVar);
/* Display measured and calculation data (on UART) */
void CN0395_DisplayData(sMeasurementVariables *sMeasVar);
/* Display info for <help> command */
void CN0395_CmdHelp(uint8_t *args, sMeasurementVariables *sMeasVar);
/* Run the calibration routine and display the calibration correction factor */
void CN0395_CmdCalibration(uint8_t *args, sMeasurementVariables *sMeasVar);
/* Update operation mode according to operation command: RH, RS or RO */
void CN0395_CmdSetOpMode(uint8_t *args, sMeasurementVariables *sMeasVar);
/* Routine for Setting Heater Current to Constant Current  IH */
void CN0395_CmdSetHeaterCurrent(uint8_t *args, sMeasurementVariables *sMeasVar);
/* Function for <voltage> command which calls the routine for setting heater voltage */
void CN0395_CmdSetHeaterVoltage(uint8_t *args, sMeasurementVariables *sMeasVar);
/* Function for <resistance> command which calls the routine for setting heater resistance */
void CN0395_CmdSetHeaterRes(uint8_t *args, sMeasurementVariables *sMeasVar);
/*  Routine for Setting Heater Temperature to Constant Temperature TH  */
void CN0395_CmdSetHeaterTemp(uint8_t *args, sMeasurementVariables *sMeasVar);
/* Read ADC data */
uint16_t CN0395_ReadAdc(sMeasurementVariables *sMeasVar);
/* Calculate RS value */
float CN0395_CalculateRs(uint16_t ui16Adcdata, uint32_t ui32RgValue);
/* Calculate gas concentration in PPM */
float CN0395_CalculatePPM(sMeasurementVariables *sMeasVar);
/* Finds the next command line argument */
uint8_t *CN0395_FindArgv(uint8_t *args);
/* Separates a command line argument */
void CN0395_GetArgv(char *dst, uint8_t *args);

#endif
