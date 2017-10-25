/*!
 *****************************************************************************
 * @file:    Cmd_calibrate.c
 * @brief:   Calibration methods (from a command line)
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

/***************************** Include Files **********************************/
#include <cstdio>
#include <cfloat>
#include <cmath>

#include <Cmd_calibrate.h>
#include <Cmd.h>

#include <ADC.h>
#include <CN0338.h>
#include <Flash.h>

/****************************** Global variables *****************************/

CmdMbllCalibrate CmdMbllCalibrate;
CmdSbllCalibrate CmdSbllCalibrate;

CMD_MAP(mbllcalibrate, &CmdMbllCalibrate, "  Calibration using modified Beer-Lambert law");
CMD_MAP(sbllcalibrate, &CmdSbllCalibrate, "  Calibration using standard Beer-Lambert law");

/****************************** Global functions *****************************/

/**
   @brief Perform calibration using modified Beer-Lambert law (when <ENTER> is pressed)

   @return none
**/
void CmdMbllCalibrate::on_Enter(void)
{
   switch (status) {
   case input_b:
      b = FLT_MAX;

      scanf("%f", &b);

      if ((b < MIN_B_VALUE) || (b > MAX_B_VALUE)) {
         auto min = MIN_B_VALUE, max = MAX_B_VALUE;

         printf("\nCoefficient 'b' should be a float value: %f<=b<= %f\r\n", min, max);
         printf("Please input 'b': ");

      } else {
         status = input_c;
         scanf("%*[^][]");
         on_Enter();
      }

      break;

   case input_c:
      c = FLT_MAX;

      scanf("%f", &c);

      if ((c < MIN_C_VALUE) || (c > MAX_C_VALUE)) {
         auto min = MIN_C_VALUE, max = MAX_C_VALUE;

         printf("\nCoefficient 'c' should be a float value: %f<=c<=%f\r\n", min, max);
         printf("Please input 'c': ");

      } else {
         status = input_x1;
         scanf("%*[^][]");
         on_Enter();
      }

      break;

   case input_x1:
      x1 = FLT_MAX;

      scanf("%f", &x1);

      if ((x1 < MIN_CONCENTRATION) || (x1 > MAX_CONCENTRATION)) {
         auto min = MIN_CONCENTRATION, max = MAX_CONCENTRATION;

         printf("\nCO2 concentration(%c) 'x_low' should be a float value: %f<=x_low<= %f\r\n", 0x25, min, max);
         printf("Please input 'x_low': ");

      } else {
         status = calibrate_x1;
      }

      break;

   case input_x2:
      x2 = FLT_MAX;

      scanf("%f", &x2);

      if ((x2 < MIN_CONCENTRATION) || (x2 > MAX_CONCENTRATION)
            || (fabsf(x2 - x1) < (MIN_CONCENTRATION - MAX_CONCENTRATION) / 10)) {
         auto min = MIN_CONCENTRATION, max = MAX_CONCENTRATION;

         printf("\nCO2 concentration(%c) 'x_cal' should be a float value: %f<=x_cal<= %f and not close to 'x_low'!\r\n", 0x25,
                min, max);
         printf("Please input 'x_cal': ");

      } else {
         status = calibrate_x2;
      }

      break;

   case calibrate_x1:
   case calibrate_x2:
      puts("\nPlease wait!\r");

   default:
      break;
   }

   fflush(stdout);

   scanf("%*[^][]");
}

/**
   @brief Perform calibration using modified Beer-Lambert law (when <Ctrl + 'c'> combination is pressed)

   @return none
**/
void CmdMbllCalibrate::on_Ctrl_c(void)
{
   status = input_b;

   puts("\r\n\nCalibration process has been canceled!\r\n");

   fflush(stdout);

   scanf("%*[^][]");

   pActiveTarget = &Prompt;
   pActiveTarget->on_Enter();
}

/**
   @brief Perform calibration using modified Beer-Lambert law (when NDIR occur)

   @return none
**/
void CmdMbllCalibrate::on_NDIR(void)
{
   SOptions Options;

   switch (status) {
   case calibrate_x1:
      I1 = (float) (adc0high_data - adc0low_data) / (float) (adc1high_data - adc1low_data);
      k1 = CN0338_GetKelvin();
      status = input_x2;
      on_Enter();
      break;

   case calibrate_x2:
      I2 = (float) (adc0high_data - adc0low_data) / (float) (adc1high_data - adc1low_data);
      k2 = CN0338_GetKelvin();

      x1 *= ((float)k2 / (float)k1); // calculate at k2

      float span, zero;

      span = (float)(I2 - I1) / (float)((I1 * (expf(-b * powf(x2, c)) - 1)) - (I2 * (expf(-b * powf(x1, c)) - 1)));

      zero = (float)I2 / (float)(1 + ((expf(-b * powf(x2, c)) - 1) * span));

      Options = Flash_GetOptions();
      Options.zero = zero;
      Options.span = span;
      Options.b = b;
      Options.c = c;
      Options.k = k2;

      Flash_SetOptions(Options);

      puts("\nSuccessful calibrated!\r\n");

      fflush(stdout);

      scanf("%*[^][]");

      status = input_b;

      pActiveTarget = &Prompt;
      pActiveTarget->on_Enter();
      break;

   default:
      break;
   }
}

/**
   @brief Perform calibration using standard Beer-Lambert law (when <ENTER> is pressed)

   @return none
**/
void CmdSbllCalibrate::on_Enter(void)
{
   switch (status) {
   case input_x1:
      x1 = FLT_MAX;

      scanf("%f", &x1);

      if ((x1 < MIN_CONCENTRATION) || (x1 > MAX_CONCENTRATION)) {
         auto min = MIN_CONCENTRATION, max = MAX_CONCENTRATION;

         printf("\nCO2 concentration(%c) 'x_low' should be a float value: %f<=x_low<= %f\r\n", 0x25, min, max);
         printf("Please input 'x_low': ");

      } else {
         status = calibrate_x1;
      }

      break;

   case input_x2:
      x2 = FLT_MAX;

      scanf("%f", &x2);

      if ((x2 < MIN_CONCENTRATION) || (x2 > MAX_CONCENTRATION)
            || (fabsf(x2 - x1) < (MIN_CONCENTRATION - MAX_CONCENTRATION) / 10)) {
         auto min = MIN_CONCENTRATION, max = MAX_CONCENTRATION;

         printf("\nCO2 concentration(%c) 'x_cal' should be a float value: %f<=x_cal<= %f and not close to 'x_low'!\r\n", 0x25,
                min, max);
         printf("Please input 'x_cal': ");

      } else {
         status = calibrate_x2;
      }

      break;

   case calibrate_x1:
   case calibrate_x2:
      puts("please wait!\r");

   default:
      break;
   }

   fflush(stdout);

   scanf("%*[^][]");
}

/**
   @brief Perform calibration using standard Beer-Lambert law (when <Ctrl + 'c'> combination is pressed)

   @return none
**/
void CmdSbllCalibrate::on_Ctrl_c(void)
{
   status = input_x1;

   puts("\r\n\nCalibration process has been canceled!\r\n");

   fflush(stdout);

   scanf("%*[^][]");

   pActiveTarget = &Prompt;
   pActiveTarget->on_Enter();
}

/**
   @brief Perform calibration using standard Beer-Lambert law (when NDIR occur)

   @return none
**/
void CmdSbllCalibrate::on_NDIR(void)
{
   SOptions Options;

   switch (status) {
   case calibrate_x1:
      I1 = (float) (adc0high_data - adc0low_data) / (float) (adc1high_data - adc1low_data);
      k1 = CN0338_GetKelvin();
      status = input_x2;
      on_Enter();
      break;

   case calibrate_x2:
      I2 = (float) (adc0high_data - adc0low_data) / (float) (adc1high_data - adc1low_data);
      k2 = CN0338_GetKelvin();

      x1 *= (k2 / k1); // calculate at k2

      float b, zero;

      b = (float)(logf(I1 / I2)) / (float)(x2 - x1);

      zero = (float)I2 / (float)(expf(-b * x2));

      Options = Flash_GetOptions();
      Options.zero = zero;
      Options.span = 1.f;
      Options.b = b;
      Options.c = 1.f;
      Options.k = k2;

      Flash_SetOptions(Options);

      puts("\nSuccessful calibrated!\r\n");

      fflush(stdout);

      scanf("%*[^][]");

      status = input_x1;

      pActiveTarget = &Prompt;
      pActiveTarget->on_Enter();
      break;

   default:
      break;
   }
}
