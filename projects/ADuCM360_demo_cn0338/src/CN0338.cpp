/**
******************************************************************************
*   @file     CN0338.cpp
*   @brief    Source file for CN0338 application
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
#include <cmath>
#include <cassert>
#include <climits>
#include <ctime>
#include <time.h>

#include "CN0338.h"
#include "Timer.h"
#include <ADC.h>
#include <Cmd.h>
#include <Communication.h>
#include <Flash.h>

#include <UrtLib.h>
#include <IntLib.h>
#include <DioLib.h>
#include <AdcLib.h>


/****************************** Static functions *****************************/

/**
   @brief Interrupt initialization

   @return none
**/
static void CN0338_NVICInit(void)
{
   NVIC_SetPriorityGrouping(PRIGROUP);

   NVIC_SetPriority(UART_IRQn, NVIC_EncodePriority(PRIGROUP, 0x2, 0x0));
   NVIC_SetPriority(ADC0_IRQn, NVIC_EncodePriority(PRIGROUP, 0x4, 0x0));
   NVIC_SetPriority(ADC1_IRQn, NVIC_EncodePriority(PRIGROUP, 0x4, 0x0));

   NVIC_EnableIRQ(UART_IRQn);
   NVIC_EnableIRQ(ADC0_IRQn);
   NVIC_EnableIRQ(ADC1_IRQn);

}

/**
   @brief CN0338 application initialization

   @return none
**/
void CN0338_Init(void)
{

   Flash_Init();

   DioCfgPin(pADI_GP1, PIN0, 0);
   DioPulPin(pADI_GP1, PIN0, 1);
   DioOenPin(pADI_GP1, PIN0, 1);

   ADC_Init();

   SOptions defaultOptions, Options;


   if (Flash_VerifyOptions()) {
      Options = Flash_GetOptions();
      Options.baud_rate = defaultOptions.baud_rate;
      Flash_SetOptions(Options);
      UART_Init(Options.baud_rate);

   } else {
      SOptions defaultOptions;
      UART_Init(defaultOptions.baud_rate);
   }

   CN0338_NVICInit();

}


/**
   @brief Toggle LDO pin

   @return none
**/
void CN0338_ToggleLDO(void)
{
   DioTgl(pADI_GP1, GP1TGL_TGL0);
}

float CN0338_GetConcentration(void)
{
   float I = (float)(adc0high_data - adc0low_data) / (float)(adc1high_data - adc1low_data);

   float fa = 1 - ((float)I / (float)Flash_GetOptions().zero);

   float x = powf(logf(1 - ((float)fa / (float)Flash_GetOptions().span)) / (float)(-Flash_GetOptions().b), 1 / (float)Flash_GetOptions().c); // please see circuit note

   x *= ((float)CN0338_GetKelvin() / (float)Flash_GetOptions().k);
   return x;
}

using namespace std;

/**
   @brief Calculate temperature (kelvin)

   @return float - temperature value
**/
float CN0338_GetKelvin(void)
{
   auto temperature = (2.349422e7f
                       / (7.88e4f + 5.963e3f * logf((1.20003e5f * ntc_data) / (1.217287684096e13f - 1.1584e5f * ntc_data))));

   return temperature;
}

/**
   @brief Convert teperature from kelvin to celsius

   @return float - temperature value
**/
float CN0338_KelvinToCelsius(float kelvin)
{
   return (kelvin - 273.15f);
}

/**
   @brief Calculate temperature (celsius)

   @return float - temperature value
**/
float CN0338_GetCelsius(void)
{
   return CN0338_KelvinToCelsius(CN0338_GetKelvin());
}

/**
   @brief At NDIR event

   @return none
**/
void on_NDIR(void)
{
   Timer_add(roundf(500.f / Flash_GetOptions().ndir_frequency), on_NDIR);

   pActiveTarget->on_NDIR();

   CN0338_ToggleLDO();
}

/**
   @brief NDIR initialization

   @return none
**/
void CN0338_InitNDIR(void)
{
   Timer_add(roundf(500.f / Flash_GetOptions().ndir_frequency), on_NDIR);
}
