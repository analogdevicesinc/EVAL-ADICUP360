/**
******************************************************************************
*   @file     Cmd_settings.cpp
*   @brief    Source file for settings application parameters (from command line)
*   @version  V0.1
*   @author   ADI
*   @date     February 2016
*  @par Revision History:
*  - V0.1, February 2016: initial version.
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

/***************************** Include Files **********************************/
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cfloat>
#include <cmath>

#include "Cmd.h"
#include "Cmd_Settings.h"

#include <ADuCM360.h>
#include <AdcLib.h>

#include "Flash.h"
#include "ADC.h"
#include "Communication.h"
#include "CN0338.h"

/****************************** Global variables *****************************/

CmdPrintSetting CmdPrintSetting;
CmdSetADC CmdSetADC;
CmdSetBaudRate CmdSetBaudRate;
CmdSetFalling CmdSetFalling;
CmdSetNDIR CmdSetNDIR;
CmdSetRising CmdSetRising;

CMD_MAP(printsettings, &CmdPrintSetting, "  Display system settings\t\t");
CMD_MAP(setadc, &CmdSetADC, "  Set ADC sample frequency\t\t");
CMD_MAP(setbaudrate, &CmdSetBaudRate, "  Set serial port baud rate value\t");
CMD_MAP(setfalling, &CmdSetFalling, "  Set falling edge time for NDIR signal");
CMD_MAP(setndir, &CmdSetNDIR, "  Set NDIR light source frequency\t");
CMD_MAP(setrising, &CmdSetRising, "  Set rising edge time for NDIR signal\t");

/****************************** Global functions *****************************/

/**
   @brief Print application settings (when <ENTER> is pressed)

   @return none
**/
void CmdPrintSetting::on_Enter(void)
{
   scanf("%*[^][]");

   SOptions defaultOptions, Options = Flash_GetOptions();

   printf("\n");


   printf("\t - Current baud rate = %uHz; ", Options.baud_rate);
   printf("Default value = %uHz\r\n", defaultOptions.baud_rate);

   printf("\t - Current NDIR light source frequency = %fHz;", Options.ndir_frequency);
   printf("Default = %fHz\r\n", defaultOptions.ndir_frequency);

   printf("\t - Current NDIR rising edge time = %ums, ", Options.rising_ms);
   printf("Default = %ums\r\n", defaultOptions.rising_ms);

   printf("\t - Current NDIR falling edge time = %ums, ", Options.falling_ms);
   printf("Default = %ums\r\n", defaultOptions.falling_ms);

   printf("\t - Current ADC sample frequency = %fHz, ", Options.adc_frequency);
   printf("Default = %fHz\r\n", defaultOptions.adc_frequency);

   printf("\t - Current coefficient 'ZERO' value = %f, ", Options.zero);
   printf("Default = %f\r\n", defaultOptions.zero);

   printf("\t - Current coefficient 'SPAN' value= %f, ", Options.span);
   printf("Default = %f\r\n", defaultOptions.span);

   printf("\t - Current coefficient 'b' value= %f, ", Options.b);
   printf("Default = %f\r\n", defaultOptions.b);

   printf("\t - Current coefficient 'c' value = %f, ", Options.c);
   printf("Default = %f\r\n", defaultOptions.c);

   printf("\t - Current calibrated temperature 'k' = %f˚C, ", CN0338_KelvinToCelsius(Options.k));
   printf("Default = %f˚C\r\n", CN0338_KelvinToCelsius(defaultOptions.k));
   printf("\n");

   fflush(stdout);

   pActiveTarget = &Prompt;
   pActiveTarget->on_Enter();
}

/**
   @brief Set ADC frequency (when <ENTER> is pressed)

   @return none
**/
void CmdSetADC::on_Enter(void)
{
   auto adc_frequency = FLT_MAX;

   scanf("%f", &adc_frequency);

   if ((adc_frequency < MIN_ADC_FREQ) || (adc_frequency > MAX_ADC_FREQ)) {
      auto min = MIN_ADC_FREQ, max = MAX_ADC_FREQ;

      printf("\nADC sample frequency should be a float value: %f<='ADC frequency'<=%f.\r\n", min, max);
      printf("Please input ADC frequency [Hz]: ");

      fflush(stdout);
      scanf("%*[^][]");

   } else {
      int AF, SF;

      adc_frequency = ADC_FindFactor(AF, SF, adc_frequency);

      SOptions Options = Flash_GetOptions();
      Options.adc_frequency = adc_frequency;

      printf("\n\tADC sample frequency = %fHz, AF=%i, SF=%i\r\n\n", adc_frequency, AF, SF);

      AdcFlt(pADI_ADC0, SF, AF, ADCFLT_NOTCH2 | ADCFLT_RAVG2 | ADCFLT_CHOP);
      AdcFlt(pADI_ADC1, SF, AF, ADCFLT_NOTCH2 | ADCFLT_RAVG2 | ADCFLT_CHOP);

      if (adc_frequency < (30.f * Options.ndir_frequency)) {
         Options.ndir_frequency = adc_frequency / 30.f;

         puts("\n\033[4mNOTE\033[0m: ADC sample frequency need to be greater than NDIR frequency 30 times:\r\n");
         printf("\tNDIR frequency = %fHz.\r\n\n", Options.ndir_frequency);
      }

      Flash_SetOptions(Options);

      fflush(stdout);
      scanf("%*[^][]");

      pActiveTarget = &Prompt;
      pActiveTarget->on_Enter();
   }
}

/**
   @brief Set UART baud rate (when <ENTER> is pressed)

   @return none
**/
void CmdSetBaudRate::on_Enter(void)
{
   auto baud_rate = 0u;

   scanf("%u", &baud_rate);

   if ((baud_rate < MIN_BAUD_RATE) || (baud_rate > MAX_BAUD_RATE)) {
      auto min = MIN_BAUD_RATE, max = MAX_BAUD_RATE;
      printf("\nBaud rate should be an integer value: %i, 38400, 57600, 115200, 230400 or  %i.\r\n", min, max);
      printf("Please input baud rate [Hz]: ");
      fflush(stdout);
      scanf("%*[^][]");

   } else {
      SOptions Options = Flash_GetOptions();
      Options.baud_rate = baud_rate;

      printf("\n\tBaud rate = %dHz\r\n\n", Options.baud_rate);

      scanf("%*[^][]");

      Flash_SetOptions(Options);

      UART_Init(Options.baud_rate);

      pActiveTarget = &Prompt;
      pActiveTarget->on_Enter();
   }
}

/**
   @brief Set falling edge time for NDIR signal (when <ENTER> is pressed)

   @return none
**/
void CmdSetFalling::on_Enter(void)
{
   auto falling_ms = UINT_MAX;

   scanf("%u", &falling_ms);

   if (falling_ms > MAX_FALLING_EDGE) {
      decltype(falling_ms) max = MAX_FALLING_EDGE;

      printf("\nFalling edge time of NDIR signal should be an integer value: 0<='time'<= %u.\r\n", max);
      printf("Please input the falling time [ms]: ");
      fflush(stdout);
      scanf("%*[^][]");

   } else {
      SOptions Options = Flash_GetOptions();
      Options.falling_ms = falling_ms;

      printf("\n\tFalling edge time = %ims\r\n\n", Options.falling_ms);

      if (falling_ms > (250.f / Options.ndir_frequency)) {
         Options.ndir_frequency = (250.f / falling_ms);

         puts("\n\033[4mNOTE\033[0m: Falling edge time should be shorter than 1/4 of NDIR period:\r\n");
         printf("\tNDIR frequency =  %fHz.\r\n\n", Options.ndir_frequency);
      }

      Flash_SetOptions(Options);

      fflush(stdout);
      scanf("%*[^][]");

      pActiveTarget = &Prompt;
      pActiveTarget->on_Enter();
   }
}

/**
   @brief Set NDIR light source frequency(when <ENTER> is pressed)

   @return none
**/
void CmdSetNDIR::on_Enter(void)
{
   auto ndir_frequency = FLT_MAX;

   scanf("%f", &ndir_frequency);

   if ((ndir_frequency < MIN_NDIR_FREQ) || (ndir_frequency > MAX_NDIR_FREQ)) {
      auto min = MIN_NDIR_FREQ, max = MAX_NDIR_FREQ;

      printf("\nNDIR light source frequency should be a float value: %f<='frequency'<=%f.\r\n", min, max);
      printf("Please input NDIR frequency [Hz]: ");

   } else {
      SOptions Options = Flash_GetOptions();
      Options.ndir_frequency = ndir_frequency;

      printf("\n\tNDIR light source frequency = %fHz\r\n\n", Options.ndir_frequency);

      if (ndir_frequency > (Options.adc_frequency / 30.f)) {
         int AF, SF;

         for (auto adc_frequency = ndir_frequency * 30.f; ndir_frequency > (Options.adc_frequency / 30.f); adc_frequency *=
                  1.1f) {
            Options.adc_frequency = ADC_FindFactor(AF, SF, adc_frequency);
         }

         puts("\n\033[4mNOTE\033[0m: ADC sample frequency need to be greater than 30 times of NDIR frequency:\r\n");
         printf("\tADC sample frequency = %fHz, AF=%i, SF=%i\r\n\n", Options.adc_frequency, AF, SF);

         pADI_ADC0->FLT = ADC0FLT_CHOP_ON | ADC0FLT_RAVG2_ON | ADC0FLT_SINC4EN_DIS | (AF << 8) | ADC0FLT_NOTCH2_EN | SF; // ADC0FLT must be configure same with ADC1FLT in simultaneous sampling mode
         pADI_ADC1->FLT = ADC1FLT_CHOP_ON | ADC1FLT_RAVG2_ON | ADC1FLT_SINC4EN_DIS | (AF << 8) | ADC1FLT_NOTCH2_EN | SF;
      }

      if (ndir_frequency > (250.f / Options.rising_ms)) {
         Options.rising_ms = floorf(250.f / ndir_frequency);

         puts("\n\033[4mNOTE\033[0m: Rising edge time of the NDIR signal should be shorter than 1/4 of NDIR period:\r\n");
         printf("\tRising edge time = %ums\r\n\n", Options.rising_ms);
      }

      if (ndir_frequency > (250.f / Options.falling_ms)) {
         Options.falling_ms = floorf(250.f / ndir_frequency);

         puts("\n\033[4mNOTE\033[0m: Falling edge time of the NDIR signal should be shorter than 1/4 of NDIR period:\r\n");
         printf("\tFalling edge time = %ums\r\n\n", Options.falling_ms);
      }

      Flash_SetOptions(Options);

      fflush(stdout);
      scanf("%*[^][]");

      pActiveTarget = &Prompt;
      pActiveTarget->on_Enter();
   }
}

/**
   @brief Set rising edge time for NDIR signal (when <ENTER> is pressed)

   @return none
**/
void CmdSetRising::on_Enter(void)
{
   auto rising_ms = UINT_MAX;

   scanf("%u", &rising_ms);

   if (rising_ms > MAX_RISING_EDGE) {
      decltype(rising_ms) max = MAX_RISING_EDGE;

      printf("\nRising edge time of NDIR signal should be an integer value: 0<='time'<= %u.\r\n", max);
      printf("Please input the rising time [ms]: ");
      fflush(stdout);
      scanf("%*[^][]");

   } else {
      SOptions Options = Flash_GetOptions();
      Options.rising_ms = rising_ms;

      printf("\n\tRising edge time = %ims\r\n\n", Options.rising_ms);

      if (rising_ms > (250.f / Options.ndir_frequency)) {
         Options.ndir_frequency = (250.f / rising_ms);

         puts("\n\033[4mNOTE\033[0m: Rising edge time should be shorter than 1/4 of NDIR period:\r\n");
         printf("\tNDIR frequency is set to: %fHz.\r\n\n", Options.ndir_frequency);
      }

      Flash_SetOptions(Options);

      fflush(stdout);
      scanf("%*[^][]");

      pActiveTarget = &Prompt;
      pActiveTarget->on_Enter();
   }
}

