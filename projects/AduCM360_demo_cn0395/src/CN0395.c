/*!
 *****************************************************************************
 * @file:    CN0395.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2017 Analog Devices, Inc.

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

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "UrtLib.h"
#include "ClkLib.h"
#include <FeeLib.h>

#include <ADUCM360.h>
#include <CN0395.h>
#include "Communication.h"
#include "Timer.h"

#include <ADN8810.h>
#include <AD7988.h>
#include <SHT30.h>
#include <Flash.h>

/************************* Variable Definitions ******************************/

uint32_t RsTime = 0;
uint8_t ui8ContinousRsMeasurement = 0;

/* Available commands */
char *CmdCommands[] = {
   "help",
   "calibration",
   "operation",
   "current",
   "voltage",
   "resistance",
   "temperature",
   ""
};

/* Functions for available commands */
cmdFunc CmdFun[] = {
   CN0395_CmdHelp,
   CN0395_CmdCalibration,
   CN0395_CmdSetOpMode,
   CN0395_CmdSetHeaterCurrent,
   CN0395_CmdSetHeaterVoltage,
   CN0395_CmdSetHeaterRes,
   CN0395_CmdSetHeaterTemp,
   NULL
};

/************************* Functions Definitions ******************************/

/**
 * @brief Calls the init functions for SPI, I2C and UART
 *
 * @param  None.
 *
 * @return  None.
**/
void CN0395_Init(void)
{
   /* Initialize SPI */
   SPI_Init();
   /* Initialize I2C */
   I2C_Init();
   /* Initialize UART port */
   UART_Init (B9600, COMLCR_WLS_8BITS);
}

/**
   @brief Command line process function

   @return none
**/
void CN0395_CmdProcess(sMeasurementVariables *sMeasVar)
{
   cmdFunc   func;

   func = CN0395_FindCommand((char *)uart_rx_buffer);    /* Find needed function based on typed command */

   if (func) {                                           /* Check if there is a valid command */
      AppPrintf("\n");
      (*func)(&uart_rx_buffer[2], sMeasVar);             /* Call the desired function */
   }
   else if (strlen((char *)uart_rx_buffer) != 0) {       /* Check if there is no match for typed command */
      AppPrintf("\n");
      AppPrintf("Unknown command!");                     /* Display a message for unknown command */
      AppPrintf("\n");
   }
   else { // if just <ENTER> is pressed, make a new measurement and display RS data
         ui8ContinousRsMeasurement ^= 1;
   }
}

/**
   @brief Find available commands

   @param cmd - command to search

   @return cmdFunc - return the specific function for available command or NULL for invalid command
**/
cmdFunc CN0395_FindCommand(char *cmd)
{
   cmdFunc func = NULL;
   int i = 0;

   while (CmdFun[i] != NULL) {
      if (strncmp(cmd, CmdCommands[i], 6) == 0) {
         func = CmdFun[i];
         break;
      }
      i++;
   }

   return func;
}

/**
   @brief Command Prompt function

   @return res - UART_SUCCESS
**/
int CN0395_CmdPrompt(void)
{
   int res;

   res = UART_WriteChar(_CR, UART_WRITE_NO_INT);

   if (res == UART_SUCCESS) {
      res = UART_WriteChar(_LF, UART_WRITE_NO_INT);
   }

   if (res == UART_SUCCESS) {
      UART_WriteChar('>', UART_WRITE_NO_INT);
   }
   uart_rcnt = 0;

   return res;
}

/**
   @brief Compute RH, PH and TH and update sMeasVar struct with the new values

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return None.
**/
void CN0395_ComputeHeaterRPT(sMeasurementVariables *sMeasVar)
{
   float    fHeaterVoltage;
   float    fHeaterRes;
   float    fAmbientHeaterTemp;
   float    fAmbientHeaterHum;
   float    fHeaterCurrent;
   float    fHeaterPower;
   float    fHeaterTemp;

   const float fHeaterNomimalRes = 110; // RH_0 = 110Ω @ 20°C

   fHeaterVoltage = sMeasVar->fHeaterVoltage;
   fHeaterCurrent = sMeasVar->fHeaterCurrent;
   // Calculate RH = VH/IH
   fHeaterRes = (fHeaterVoltage / fHeaterCurrent) * pow(10, 3); // Ω

   // Calculate PH = VH*IH
   fHeaterPower = fHeaterVoltage * fHeaterCurrent; // mW

   // Read T_A and HUM from temperature/humidity sensor
   SHT30_Update(&fAmbientHeaterTemp, &fAmbientHeaterHum);

   /* RH_T = RH_A [ 1 + ALPHA*(RH_0/RH_A)*(T – T_A)] - solve for T as a function of RH_T:
    * T =  (RH_T – RH_A) / (ALPHA * RH_0) + T_A
    */
   fHeaterTemp = (fHeaterRes - sMeasVar->fAmbientHeaterRes) / (ALPHA * fHeaterNomimalRes) + fAmbientHeaterTemp;

   sMeasVar->fHeaterRes = fHeaterRes;
   sMeasVar->fAmbientHeaterTemp = fAmbientHeaterTemp;
   sMeasVar->fAmbientHeaterHum = fAmbientHeaterHum;
   sMeasVar->fHeaterPower = fHeaterPower;
   sMeasVar->fHeaterTemp = fHeaterTemp;
}

/**
   @brief After initialization, circuit resets then measures heater ambient temperature T_A,
          humidity HUM, then ambient temperature heater resistance RH_A. Save T_A, HUM, RH_A.
          Also reads the correction factor (K1) from the permanent memory.
          Moreover, it is assumed that init is performed in clean air, therefore we measure Ro.

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return None.
**/
void CN0395_PowerOn(sMeasurementVariables *sMeasVar)
{
   uint16_t ui16AdcData;
   float    fHeaterVoltage;
   float    fAmbientHeaterRes;
   float    fAmbientHeaterTemp;
   float    fAmbientHeaterHum;
   uint32_t ui32CalibrationData[2];

   // Enable master power on IO5 and reset DAC
   ADN8810_Init();

   // Switch to RH mode and disable RS switches, delay 50ms
   AD7988_SetOperationMode(AD7988_RH_MODE);
   sMeasVar->OpMode = AD7988_RH_MODE;

   // Read from permanent memory the calibration data
   ReadFromFlash(ui32CalibrationData, 0x1f000, sizeof(ui32CalibrationData));
   if (ui32CalibrationData[0] == 1) { // first element of the array is the information if calibration has been performed before
         // The second element of the array is the gain calibration factor which needs to be converted from uint32_t to float
         sMeasVar->K1 = (float)(ui32CalibrationData[1] / 10000) + (float)(ui32CalibrationData[1] % 10000) / 10000;
   }
   else {
      // In the case that Factory Calibration has never been performed before, use gain calibration factor of 1
      sMeasVar->K1 = 1;
   }

   timer_sleep(50);

   // Set default heater current to 8mA
   sMeasVar->fHeaterCurrent = 8;
   ADN8810_SetOutput(sMeasVar->fHeaterCurrent, sMeasVar);

   // Delay 20uS; this is long enough for DAC current to settle, but not long enough for heater temp to change
   timer_sleep_5uS(4);

   ui16AdcData = CN0395_ReadAdc(sMeasVar);
   fHeaterVoltage = AD7988_DataToVoltage(ui16AdcData);
   // Calculate RH_A = VH_A / 8mA
   fAmbientHeaterRes = (fHeaterVoltage / sMeasVar->fHeaterCurrent) * pow(10, 3);

   // Read T_A and HUM from temperature/humidity sensor
   SHT30_Update(&fAmbientHeaterTemp, &fAmbientHeaterHum);

   // Store some init measurement values for future calculations
   sMeasVar->fAmbientHeaterRes = fAmbientHeaterRes;
   sMeasVar->fHeaterVoltage = fHeaterVoltage;
   sMeasVar->fAmbientHeaterTemp = fAmbientHeaterTemp;
   sMeasVar->fAmbientHeaterHum = fAmbientHeaterHum;

   CN0395_ComputeHeaterRPT(sMeasVar); // Compute RH, PH and TH

   // At power up, perform a Ro measurement and store it for future use
   sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   while (sMeasVar->fSensorResCleanAir < 0) { // in case of error, repeat measurement
         timer_sleep(500);
         sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   }
}

/**
   @brief Routine for measuring sensor resistance RS:
          - Set IO6 to logic 1 enable switches
          - Set IN1 to logic 0 to connect ADC to RS sensor resistor
          - Use the gain ranging algorithm to determine the value of RS

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return None.
**/
float CN0395_MeasureSensorResistance(sMeasurementVariables *sMeasVar)
{
   float    fRS = 0;
   float    VS;
   uint16_t ui16AdcData;
   uint32_t ui32RgValue = 0;
   int8_t  i8GainRangingIndex = AD7988_RS_RAMGE_5;
   uint32_t GainRangingTh[5][4] = {
            {442, 8870, 1922, 132},
            {393, 39200, 10263, 3330},
            {3468, 110000, 56753, 84000},
            {3492, 2740000, 57175, 2119971},
            {7171, 33300000, 62415, 1909936}
   };

   // Gain ranging algorithm, needs to be run with every sensor measurement
   while (i8GainRangingIndex >= AD7988_RS_RAMGE_1) {

         AD7988_SetOperationMode(i8GainRangingIndex);
         sMeasVar->OpMode = i8GainRangingIndex;

         ui16AdcData = CN0395_ReadAdc(sMeasVar);

         if(ui16AdcData > GainRangingTh[i8GainRangingIndex][2]) {
               AppPrintf("\r\nADC Data Error  = %d ", ui16AdcData);
               return -1;
         }
         // ADC Data is bellow the threshold switch to the next gain
         else if (ui16AdcData < GainRangingTh[i8GainRangingIndex][0]) {
               i8GainRangingIndex--;
         }
         else { // ADC read data is in the correct range --> calculate RS
               AD7988_SetOperationMode(AD7988_RH_MODE);
               ui32RgValue = GainRangingTh[i8GainRangingIndex][1];
               fRS = CN0395_CalculateRs(ui16AdcData, ui32RgValue);
               VS = AD7988_DataToVoltage(ui16AdcData);
               sMeasVar->fSensorVoltage = VS;
               sMeasVar->ui32GainResistance = ui32RgValue;
               sMeasVar->fSensorRes = fRS;
               sMeasVar->fPPM = CN0395_CalculatePPM(sMeasVar);

               if (fRS > GainRangingTh[i8GainRangingIndex][3]) { // RS exceeds max value
                     return -1;
               }

               return fRS;
         }
   }
   return 0; // never reached
}

/**
 * @brief Display measured and calculation data (on UART).
 *
 * @param sMeasVar - pointer to the struct that contains all the relevant measurement variables.

 * @return None.
**/
void CN0395_DisplayData(sMeasurementVariables *sMeasVar)
{
   char *percent = "%";

   if(sMeasVar->OpMode == AD7988_RH_MODE) {
         AppPrintf("\r\nAvailable data for RH mode:");
         UART_WriteString("\r\n");
         AppPrintf("\r\nAmbient Heater Res:  RH_A  = %.2f [Ohms]", sMeasVar->fAmbientHeaterRes);
         AppPrintf("\r\nHeater Voltage:      VH    = %.4f [V]", sMeasVar->fHeaterVoltage);
         AppPrintf("\r\nHeater Current:      IH    = %.4f [mA]", sMeasVar->fHeaterCurrent);
         AppPrintf("\r\nHeater Resistance:   RH    = %.2f [Ohms]", sMeasVar->fHeaterRes);
         AppPrintf("\r\nAmbient Heater Temp: T_A   = %.2f [C]", sMeasVar->fAmbientHeaterTemp);
         AppPrintf("\r\nAmbient Heater Hum:  HUM   = %.2f [%s]", sMeasVar->fAmbientHeaterHum, percent);
         AppPrintf("\r\nHeater Power:        PH    = %.4f [mW]", sMeasVar->fHeaterPower);
         AppPrintf("\r\nHeater Temp:         TH    = %.2f [C]", sMeasVar->fHeaterTemp );
         AppPrintf("\r\nADC Data                   = 0x%x", sMeasVar->ui16LastAdcDataRead);
         AppPrintf("\r\nSensor Res Air:      Ro    = %.2f [Ohms]", sMeasVar->fSensorResCleanAir);
   }
   else {
         AppPrintf("\r\nAvailable data for RS mode:");
         UART_WriteString("\r\n");
         AppPrintf("\r\nSensor Resistance:      Rs    = %.2f [Ohms]", sMeasVar->fSensorRes);
         AppPrintf("\r\nSensor Resistance Air:  Ro    = %.2f [Ohms]", sMeasVar->fSensorResCleanAir);
         AppPrintf("\r\nGas Concentration       C     = %.2f [PPM]", sMeasVar->fPPM);
         AppPrintf("\r\nSensor Voltage:         Vs    = %.4f [V]", sMeasVar->fSensorVoltage);
         AppPrintf("\r\nHeater Voltage:         VH    = %.4f [V]", sMeasVar->fHeaterVoltage);
         AppPrintf("\r\nAmbient Heater Temp:    T_A   = %.2f [C]", sMeasVar->fAmbientHeaterTemp);
         AppPrintf("\r\nAmbient Heater Hum:     HUM   = %.2f [%s]", sMeasVar->fAmbientHeaterHum, percent);
   }
   UART_WriteString("\r\n");
}

/**
   @brief Display info for <help> command

   @param args - pointer to the arguments on the command line.

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables.

   @return none
**/
void CN0395_CmdHelp(uint8_t *args, sMeasurementVariables *sMeasVar)
{
   uint8_t        *p = args;
   char           arg[4];
   while (*(p = CN0395_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
         CN0395_GetArgv(arg, p);
   }

   AppPrintf("\n");
   AppPrintf("Available commands:\n");
   AppPrintf("\n");
   AppPrintf("Press the <ENTER> key  - Start/Stop continuous sensor measurement \n");
   AppPrintf("operation <mode>       - Set the desired operation mode: heater or sensor and perform a single measurement \n");
   AppPrintf("                         <mode> = RH, RS\n");
   AppPrintf("calibration <r/w>      - Full scale calibration routine.\n");
   AppPrintf("                         The calibration ensures that the exact ADN8810 output current is known\n");
   AppPrintf("                         and the calibration constant (K1) is stored.\n");
   AppPrintf("                         <r/w> = r read K1 stored in the permanent memory\n");
   AppPrintf("                         <r/w> = w perform a factory calibration and store K1 in memory\n");
}

/**
   @brief Run the calibration routine and display the calibration correction factor

   @param args - pointer to the arguments on the command line.

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables.

   @return none
**/
void CN0395_CmdCalibration(uint8_t *args, sMeasurementVariables *sMeasVar)
{
   uint8_t        *p = args;
   char           arg[4];
   uint32_t       ui32CalibrationData[2];
   float          K1 = 0;

   while (*(p = CN0395_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
         CN0395_GetArgv(arg, p);
   }

   // Read the correction factor K1 stored in permanent memory
   if(strncmp(arg, "r", 2) == 0) {
         ReadFromFlash(ui32CalibrationData, 0x1f000, sizeof(ui32CalibrationData));

         if(ui32CalibrationData[0] == 1) { // this array also stores the information if calibration has already been performed
               K1 = (float)(ui32CalibrationData[1] / 10000) + (float)(ui32CalibrationData[1] % 10000) / 10000;
               AppPrintf("\n");
               AppPrintf("Gain correction factor (K1) stored in memory is: %.4f", K1);
               AppPrintf("\n");
         }
         else {
               AppPrintf("\n");
               AppPrintf("Factory calibration hasn't been performed yet, default K1 value is 1");
               AppPrintf("\nType <calibrate w> to perform the factory calibration...");
               AppPrintf("\n");
         }
   }
   // Perform a new calibration routine and store K1 in permanent memory
   else if(strncmp(arg, "w", 2) == 0) {

         AppPrintf("\nPlace the P2 jumper between P2-1 and P2-2 (RCAL1) and press <c> key");
         AppPrintf("\n");

         char response;
         while (response != 'c') {
               UART_ReadChar(&response);   // Read character from UART
         }
         AppPrintf("\n");

         FeePErs(0x1f000);
         ui32CalibrationData[0] = 1; // Factory calibration has been performed, save this information in permanent memory
         ui32CalibrationData[1] = ADN8810_FactoryCalibration(); // Save gain calibreation factor in permanent memory
         WriteToFlash(ui32CalibrationData, 0x1f000, sizeof(ui32CalibrationData));

         K1 = (float)(ui32CalibrationData[1] / 10000) + (float)(ui32CalibrationData[1] % 10000) / 10000;
         sMeasVar->K1 = K1;
         AppPrintf("\n");
         AppPrintf("Gain correction factor (K1) stored in memory is: %.4f", K1);
         AppPrintf("\n");
   }
}

/**
   @brief Update operation mode according to operation command: RH, RS or RO

   @param args - pointer to the arguments on the command line.

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables.

   @return none
**/
void CN0395_CmdSetOpMode(uint8_t *args, sMeasurementVariables *sMeasVar)
{
   uint8_t        *p = args;
   char           arg[4];

   while (*(p = CN0395_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
         CN0395_GetArgv(arg, p);
   }

   if(strncmp(arg, "RH", 3) == 0) {
         sMeasVar->OpMode = AD7988_RH_MODE;
         AppPrintf("\n\tHeater mode of operation\n");
         AppPrintf("Input desired value...\n");
         AppPrintf("\n");
         AppPrintf("voltage <V>            - Constant Voltage (input voltage VH, 1.8V default)\n");
         AppPrintf("                         <V> ex: 1.8, 2, ... \n");
         AppPrintf("resistance <ohms>      - Constant Resistance (input resistance RH, 225Ohms default) \n");
         AppPrintf("                         <ohms> ex: 225, ... \n");
         AppPrintf("temperature <C>        - Constant T (input TH, 360 Celsius default)  \n");
         AppPrintf("                         <C> ex: 360, ... \n");
         AppPrintf("current <mA>           - Constant Current (Input IH, 8mA default)  \n");
         AppPrintf("                         <mA> ex: 8.49, ... \n");
   }
   else if(strncmp(arg, "RS", 3) == 0) {
         CN0395_MeasureSensorResistance(sMeasVar);
         CN0395_DisplayData(sMeasVar);
   }
}

/**
   @brief Routine for Setting Heater Current to Constant Current  IH (The default value is IH = 8mA)

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return None.
**/
void CN0395_CmdSetHeaterCurrent(uint8_t *args, sMeasurementVariables *sMeasVar)
{
   uint8_t  *p = args;
   char     arg[3];
   float    fDesiredHeaterCurrent;
   uint16_t ui16AdcData;
   float    fHeaterVoltage;

   while (*(p = CN0395_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
         CN0395_GetArgv(arg, p);
   }

   fDesiredHeaterCurrent = atof(arg); // convert string to float

   if (ADN8810_SetOutput(fDesiredHeaterCurrent, sMeasVar) > 0) { // max current is 50 mA
         AD7988_SetOperationMode(AD7988_RH_MODE);

         ui16AdcData = CN0395_ReadAdc(sMeasVar);
         fHeaterVoltage = AD7988_DataToVoltage(ui16AdcData);
         timer_sleep(50); // delay 50ms

         sMeasVar->fHeaterCurrent = fDesiredHeaterCurrent;
         sMeasVar->fHeaterVoltage = fHeaterVoltage;
         CN0395_ComputeHeaterRPT(sMeasVar); // Compute RH, PH and TH
         // Update Ro value
         sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
         while (sMeasVar->fSensorResCleanAir < 0) { // in case of error, repeat measurement
               timer_sleep(500);
               sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
         }
         sMeasVar->OpMode = AD7988_RH_MODE;
         CN0395_DisplayData(sMeasVar);
   }
   else {
         AppPrintf("\n");
         AppPrintf("Cannot set heater current (IH) to %f. Maximum full scale current is %.2f mA!", fDesiredHeaterCurrent, 50);
         AppPrintf("\n");
   }
}

/**
   @brief Routine for error correction. Depending on the enSubroutine parameter, the correction is done either for voltage or resistance

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @param fInputCurrent - input current

   @param fDesiredValue - desired VH or desired RH

   @param enSubroutine - RESISTANCE or VOLTAGE

   @return None.
**/
static void CN0395_CorrectError(sMeasurementVariables *sMeasVar,
                                float fInputCurrent,
                                float fDesiredValue,
                                enCN0395ErrCorrection enSubroutine)
{
   float    fError = 1;
   uint16_t ui16AdcData = 0;
   float    fVoltage = 0;
   float    fActualValue;

   while (fError > 0.005) { // repeat until the error is less than 0.5%
         if (ADN8810_SetOutput(fInputCurrent, sMeasVar) > 0) {
               timer_sleep(50); // delay 50ms for stabilization

               ui16AdcData = CN0395_ReadAdc(sMeasVar);
               fVoltage = AD7988_DataToVoltage(ui16AdcData); // Convert ADC data to Voltage

               if(enSubroutine == RESISTANCE) {
                     fActualValue = (fVoltage / fInputCurrent) * pow(10, 3); // Constant resistance correction
               }
               else {
                     fActualValue = fVoltage; // Constant voltage correction
               }

               if (fDesiredValue > fActualValue) {
                     fError = (fDesiredValue - fActualValue) / fDesiredValue;  // calculate the error
                     fInputCurrent = fInputCurrent + fInputCurrent * 0.5 * fError; // correct IH for the error
               }
               else {
                     fError = (fActualValue - fDesiredValue) / fDesiredValue; // calculate the error
                     fInputCurrent = fInputCurrent - fInputCurrent * 0.5 * fError; // correct IH for the error
               }
         }
         else {
               AppPrintf("\n");
               AppPrintf("Cannot set heater current (IH) to %f. Maximum full scale current is %.2f mA!", fInputCurrent, 50);
               AppPrintf("\n");
               break;
         }
   }

   sMeasVar->fHeaterVoltage = fVoltage;
   sMeasVar->fHeaterCurrent = fInputCurrent;
}


/**
   @brief Function for <voltage> command which calls the routine for setting heater voltage
          to constant voltage VH and displays the available data.

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return None.
**/
void CN0395_CmdSetHeaterVoltage(uint8_t *args, sMeasurementVariables *sMeasVar)
{
   uint8_t  *p = args;
   char     arg[3];
   float    fDesiredVoltage;
   float    fCurrent;
   const uint8_t ui8HeaterRes = 225; // 225Ω

   while (*(p = CN0395_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
         CN0395_GetArgv(arg, p);
   }
   fDesiredVoltage = atof(arg); // convert string to float
   fCurrent = (fDesiredVoltage / ui8HeaterRes) * pow(10, 3); // mA

   AD7988_SetOperationMode(AD7988_RH_MODE);

   CN0395_CorrectError(sMeasVar, fCurrent, fDesiredVoltage, VOLTAGE);

   CN0395_ComputeHeaterRPT(sMeasVar); // Compute RH, PH and TH

   // Update Ro value
   sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   while (sMeasVar->fSensorResCleanAir < 0) { // in case of error, repeat measurement
         timer_sleep(500);
         sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   }
   sMeasVar->OpMode = AD7988_RH_MODE;
   CN0395_DisplayData(sMeasVar);
}

/**
   @brief Function for <resistance> command which calls the routine for setting heater resistance
          to constant resistance RH and displays the available data.

   @param sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return None.
**/
void CN0395_CmdSetHeaterRes(uint8_t *args, sMeasurementVariables *sMeasVar)
{
   uint8_t     *p = args;
   char        arg[3];
   float       fDesiredHeaterRes;
   float       fCurrent = 8;
   const float fHeaterRes = 110; // 110Ω @ 20°C

   while (*(p = CN0395_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
         CN0395_GetArgv(arg, p);
   }

   fDesiredHeaterRes = atof(arg); // convert string to float
   // Note: The slope of the RH vs.IH curve is 115Ω/8mA = 14375Ω/A
   fCurrent = ((fDesiredHeaterRes - fHeaterRes) / 14375) * pow(10, 3);

   AD7988_SetOperationMode(AD7988_RH_MODE);

   CN0395_CorrectError(sMeasVar, fCurrent, fDesiredHeaterRes, RESISTANCE);

   CN0395_ComputeHeaterRPT(sMeasVar); // Compute RH, PH and TH

   // Update Ro value
   sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   while (sMeasVar->fSensorResCleanAir < 0) { // in case of error, repeat measurement
         timer_sleep(500);
         sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   }
   sMeasVar->OpMode = AD7988_RH_MODE;
   CN0395_DisplayData(sMeasVar);
}

/**
   @brief Routine for Setting Heater Temperature to Constant Temperature TH (The default value is TH = 360 C)

   @param *sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return None.
**/
void CN0395_CmdSetHeaterTemp(uint8_t *args, sMeasurementVariables *sMeasVar)
{
   uint8_t     *p = args;
   char        arg[3];
   float       fDesiredHeaterTemp;
   float       fAmbientHeaterTemp;
   float       fDesiredHeaterRes;
   float       fAmbientHeaterRes;
   float       fHum;
   float       fCurrent = 8;
   const float fDefaultHeaterRes = 110; // 110Ω @ 20°C

   while (*(p = CN0395_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
         CN0395_GetArgv(arg, p);
   }

   fDesiredHeaterTemp = atof(arg); // convert string to float

   SHT30_Update(&fAmbientHeaterTemp, &fHum);

   AD7988_SetOperationMode(AD7988_RH_MODE);

   fAmbientHeaterRes = sMeasVar->fAmbientHeaterRes; // get RH_A set initially at power on

   // RH_T = RH_A [ 1 + ALPHA*(RH_0/RH_A)*(T – T_A)]
   fDesiredHeaterRes = fAmbientHeaterRes *
            (1 + ALPHA * (fDefaultHeaterRes / fAmbientHeaterRes) * (fDesiredHeaterTemp - fAmbientHeaterTemp));

   CN0395_CorrectError(sMeasVar, fCurrent, fDesiredHeaterRes, RESISTANCE);

   CN0395_CorrectError(sMeasVar, fCurrent, sMeasVar->fHeaterVoltage, VOLTAGE);

   CN0395_ComputeHeaterRPT(sMeasVar); // Compute RH, PH and TH
   // Update Ro value
   sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   while (sMeasVar->fSensorResCleanAir < 0) { // in case of error, repeat measurement
         timer_sleep(500);
         sMeasVar->fSensorResCleanAir = CN0395_MeasureSensorResistance(sMeasVar);
   }
   sMeasVar->OpMode = AD7988_RH_MODE;
   CN0395_DisplayData(sMeasVar);
}

/**
   @brief Read ADC data

   @param None.

   @return ui16Adcdata - data read from ADC via SPI.
**/
uint16_t CN0395_ReadAdc(sMeasurementVariables *sMeasVar)
{
   uint16_t ui16Adcdata;

   for (uint8_t i = 0; i < 5; i++){ // Ignore the first readings until the voltage stabilizes
      AD7988_ReadData(&ui16Adcdata); // Read ADC data
   }

   sMeasVar->ui16LastAdcDataRead = ui16Adcdata;

   return ui16Adcdata;
}

/**
 * @brief Calculate RS value
 *
 * @param ui16Adcdata - ADC code
 * @param ui32RgValue - Gain resistance value
 *
 * @return fRsValue - RS value.
**/
float CN0395_CalculateRs(uint16_t ui16Adcdata, uint32_t ui32RgValue)
{
   float fRsValue;

   fRsValue = ((float)ui16Adcdata * 0.5 * (float)ui32RgValue ) / ((float)65536 - 0.5 * (float)ui16Adcdata);

   return fRsValue;
}

/**
 * @brief Calculate gas concentration in PPM
 *
 * @param *sMeasVar - pointer to the struct that contains all the relevant measurement variables
 *
 * @return fConcentration - gas concentration in PPM
**/
float CN0395_CalculatePPM(sMeasurementVariables *sMeasVar)
{
   float fConcentration = 0;
   float fRs = sMeasVar->fSensorRes;
   float fRo = sMeasVar->fSensorResCleanAir;

   if( ((fRs/fRo) >= 0.25) && ((fRs/fRo) <= 0.6) ) {
         fConcentration = 2.61 * pow((fRs/fRo), -2.63);
   }
   else if((fRs/fRo) > 0.6) {
         fConcentration = 0.550 * pow((fRs/fRo), -5.68);
   }

   return fConcentration;
}

/**
   @brief Finds the next command line argument

   @param args - pointer to the arguments on the command line.

   @return pointer to the next argument.

**/
uint8_t *CN0395_FindArgv(uint8_t *args)
{
   uint8_t  *p = args;
   int         fl = 0;

   while (*p != 0) {
      if ((*p == _SPC)) {
         fl = 1;

      } else {
         if (fl) {
            break;
         }
      }

      p++;
   }

   return p;
}

/**
   @brief Separates a command line argument

    @param dst - pointer to a buffer where the argument will be copied
    @param args - pointer to the current position of the command line .

   @return none

**/
void CN0395_GetArgv(char *dst, uint8_t *args)
{
   uint8_t  *s = args;
   char     *d = dst;

   while (*s) {
      if (*s == _SPC) {
         break;
      }

      *d++ = *s++;
   }

   *d = '\0';
}
