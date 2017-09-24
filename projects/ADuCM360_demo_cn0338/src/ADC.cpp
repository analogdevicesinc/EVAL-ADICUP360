/*!
 *****************************************************************************
 * @file:    ADC.cpp
 * @brief:   ADC handling
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
#include <cassert>
#include <cmath>
#include <cstdlib>

#include <AdcLib.h>
#include <IexcLib.h>

#include "Flash.h"
#include "ADC.h"

/********************************* Definitions ********************************/

#define ALGORITHM_PEAK2PEAK

#ifdef ALGORITHM_PEAK2PEAK
#ifdef ALGORITHM_AVERAGE
#error "both ALGORITHM_PEAK2PEAK and ALGORITHM_AVERAGE defined"
#endif
#else
#ifndef ALGORITHM_AVERAGE
#error "neither ALGORITHM_PEAK2PEAK nor ALGORITHM_AVERAGE defined"
#endif
#endif

#define ADC0CFG_SIMU_DIS (0x0<<15)
#define ADC1CFG_SIMU_DIS (0x0<<15)

#define ADC0CFG_SIMU_EN (0x1<<15)
#define ADC1CFG_SIMU_EN (0x1<<15)

#define ADC0CFG_PINSEL_DIS (0x0<<8)
#define ADC1CFG_PINSEL_DIS (0x0<<8)

#define ADC_SETTLING 2


/****************************** Global variables *****************************/

signed long adc0high_data, adc0low_data, adc1high_data, adc1low_data;

signed long ntc_data;

volatile signed long adc0_data, adc1_data;

/****************************** Function prototype *****************************/

static int ADC_AdaptPGA(signed long data, int pga);


/****************************** Global functions *****************************/

/**
   @brief Calculate AF and SF factors for ADC

   @param AF - AF factor to be stored
   @param SF - SF factor to be stored
   @param fADC - ADC frequency

   @return ADC frequency
**/
float ADC_FindFactor(int &AF, int &SF, float fADC)
{
   auto ffactor = (((1.25e5f / fADC) - 3.f) / 16.f);

   int ceil_factor = ceilf(ffactor), floor_factor = floorf(ffactor), factor = roundf(ffactor);

   while (true) {
      if ((factor % 3 == 0) && (factor / 3 <= 128) && (factor / 3 >= 1)) {
         AF = 0;
         SF = (factor / 3) - 1;
         return 1.25e5f / ((SF + 1) * 48 + 3);
      }

      for (auto af_3 = 4; af_3 <= 5; ++af_3) {
         if ((factor % af_3 == 0) && (factor / af_3 <= 127) && (factor / af_3 >= 1)) {
            AF = af_3 - 3;
            SF = (factor / af_3) - 1;
            return 1.25e5f / ((SF + 1) * 16 * af_3 + 3);
         }
      }

      for (auto af_3 = 6; af_3 <= 12; ++af_3) {
         if ((factor % af_3 == 0) && (factor / af_3 <= 126) && (factor / af_3 >= 1)) {
            AF = af_3 - 3;
            SF = (factor / af_3) - 1;
            return 1.25e5f / ((SF + 1) * 16 * af_3 + 3);
         }
      }

      for (auto af_3 = 13; af_3 <= 18; ++af_3) {
         if ((factor % af_3 == 0) && (factor / af_3 <= 125) && (factor / af_3 >= 1)) {
            AF = af_3 - 3;
            SF = (factor / af_3) - 1;
            return 1.25e5f / ((SF + 1) * 16 * af_3 + 3);
         }
      }

      if (factor == ceil_factor) {
         ++ceil_factor;
         factor = floor_factor;

      } else {
         --floor_factor;
         factor = ceil_factor;
      }
   }
}

/**
   @brief ADC initialization

   @return none
**/
void ADC_Init(void)
{
   pADI_ANA->REFCTRL = REFCTRL_REFPD_DIS;

   IexcCfg(IEXCCON_PD_En, IEXCCON_REFSEL_Int, IEXCCON_IPSEL1_Off, IEXCCON_IPSEL0_Off);

   int ADCFLT_AF, ADCFLT_SF;

   if (Flash_VerifyOptions()) {
      ADC_FindFactor(ADCFLT_AF, ADCFLT_SF, Flash_GetOptions().adc_frequency);

   } else {
      SOptions defaultOptions;

      ADC_FindFactor(ADCFLT_AF, ADCFLT_SF, defaultOptions.adc_frequency);
   }

   pADI_ADC0->PRO = ADC0PRO_ACCEN_Off | ADC0PRO_CMPEN_Off | ADC0PRO_OREN_DIS | ADC0PRO_RCEN_DIS;
   pADI_ADC1->PRO = ADC1PRO_ACCEN_Off | ADC1PRO_CMPEN_Off | ADC1PRO_OREN_DIS | ADC1PRO_RCEN_DIS;

   AdcMski(pADI_ADC0, ADCMSKI_RDY, 1);
   AdcMski(pADI_ADC1, ADCMSKI_RDY, 1);


   AdcRng(pADI_ADC0, ADCCON_ADCREF_INTREF, ADCMDE_PGA_G4 , ADCCON_ADCCODE_INT);
   AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, ADCMDE_PGA_G4 , ADCCON_ADCCODE_INT);
   AdcGo(pADI_ADC0, ADCMDE_ADCMD_CONT);
   AdcGo(pADI_ADC1, ADCMDE_ADCMD_CONT);

   AdcFlt(pADI_ADC0, ADCFLT_SF, ADCFLT_AF, ADCFLT_CHOP | ADCFLT_RAVG2 | ADCFLT_NOTCH2);
   AdcFlt(pADI_ADC1, ADCFLT_SF, ADCFLT_AF, ADCFLT_CHOP | ADCFLT_RAVG2 | ADCFLT_NOTCH2);


   AdcBuf(pADI_ADC0, ADCCFG_EXTBUF_OFF,
          ADCCON_BUFPOWN | ADCCON_BUFPOWP | ADCCON_BUFBYPP | ADCCON_BUFBYPN);
   AdcBuf(pADI_ADC1, ADCCFG_EXTBUF_OFF,
          ADCCON_BUFPOWN | ADCCON_BUFPOWP | ADCCON_BUFBYPP | ADCCON_BUFBYPN);

   AdcDiag(pADI_ADC0, ADCCON_ADCDIAG_DIAG_OFF);
   AdcDiag(pADI_ADC1, ADCCON_ADCDIAG_DIAG_OFF);

   AdcPin(pADI_ADC0, ADCCON_ADCCN_AIN3, ADCCON_ADCCP_AIN1);
   AdcPin(pADI_ADC1, ADCCON_ADCCN_AIN3, ADCCON_ADCCP_AIN2);

   pADI_ADC0->ADCCFG = ADC0CFG_SIMU_EN | ADCCFG_BOOST30_DIS | ADC0CFG_PINSEL_DIS | ADCCFG_GNDSWON_EN
                       | ADCCFG_GNDSWRESEN_DIS | ADCCFG_EXTBUF_OFF;
}


/**
   @brief Processes the ADC values in interrupt routine

   @return none
**/
void ADC_IRQ(void)
{
#ifdef ALGORITHM_AVERAGE
   static signed long long adc0_sum = 0, adc1_sum = 0;
   static auto adc0_count = 0, adc1_count = 0;
#endif

#ifdef ALGORITHM_PEAK2PEAK
   static signed long adc0_peak, adc1_peak;
#endif

   static auto adc0high_pga = 2, adc0low_pga = 2, adc1high_pga = 2, adc1low_pga = 2, ntc_pga = 2;

   static auto adc_dummy = 0;
   static auto lastLDO = GP1IN_IN0_BBA;

   if (lastLDO != GP1IN_IN0_BBA) {
#ifdef ALGORITHM_AVERAGE

      if (GP1IN_IN0_BBA) {
         adc0low_data = adc0_sum / adc0_count;
         adc1low_data = adc1_sum / adc1_count;

      } else {
         adc0high_data = adc0_sum / adc0_count;
         adc1high_data = adc1_sum / adc1_count;
      }

      adc0_sum = 0;
      adc1_sum = 0;
      adc0_count = 0;
      adc1_count = 0;
#endif

#ifdef ALGORITHM_PEAK2PEAK

      if (GP1IN_IN0_BBA) {
         adc0low_data = adc0_peak;
         adc1low_data = adc1_peak;

      } else {
         adc0high_data = adc0_peak;
         adc1high_data = adc1_peak;
      }

#endif

      AdcPin(pADI_ADC0, ADCCON_ADCCN_AIN3, ADCCON_ADCCP_AIN0);
      AdcPin(pADI_ADC1, ADCCON_ADCCN_AIN3, ADCCON_ADCCP_AIN0);

      ntc_pga = ADC_AdaptPGA(ntc_data, ntc_pga);

      AdcRng(pADI_ADC0, ADCCON_ADCREF_INTREF, (ntc_pga << 4), ADCCON_ADCCODE_INT);
      AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, (ntc_pga << 4), ADCCON_ADCCODE_INT);

      lastLDO = GP1IN_IN0_BBA;

      return;
   }

   if (adc_dummy != 0) {
      --adc_dummy;

      return;
   }

   if ((pADI_ADC0->CON & ADC0CON_ADCCP_MSK) == ADC0CON_ADCCP_AIN0) {
      ntc_data = (adc0_data + adc1_data) / 2;

      SOptions Options = Flash_GetOptions();

      AdcPin(pADI_ADC0, ADCCON_ADCCN_AIN3, ADCCON_ADCCP_AIN1);
      AdcPin(pADI_ADC1, ADCCON_ADCCN_AIN3, ADCCON_ADCCP_AIN2);

      if (GP1IN_IN0_BBA) {
         adc_dummy = floorf((Options.rising_ms * Options.adc_frequency) / 1000.f - (ADC_SETTLING * 2) - 1);

         adc0high_pga = ADC_AdaptPGA(adc0high_data, adc0high_pga);
         adc1high_pga = ADC_AdaptPGA(adc1high_data, adc1high_pga);

         AdcRng(pADI_ADC0, ADCCON_ADCREF_INTREF, (adc0high_pga << 4), ADCCON_ADCCODE_INT);
         AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, (adc1high_pga << 4), ADCCON_ADCCODE_INT);

      } else {
         adc_dummy = floorf((Options.falling_ms * Options.adc_frequency) / 1000.f - (ADC_SETTLING * 2) - 1);

         adc0low_pga = ADC_AdaptPGA(adc0low_data, adc0low_pga);
         adc1low_pga = ADC_AdaptPGA(adc1low_data, adc1low_pga);

         AdcRng(pADI_ADC0, ADCCON_ADCREF_INTREF, (adc0low_pga << 4), ADCCON_ADCCODE_INT);
         AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, (adc1low_pga << 4), ADCCON_ADCCODE_INT);
      }

      if (adc_dummy < 0) {
         adc_dummy = 0;
      }

   } else {
#ifdef ALGORITHM_AVERAGE

      if (GP1IN_IN0_BBA) {
         if (adc0high_pga == ADC_AdaptPGA(adc0_data, adc0high_pga)) {
            adc0_sum += adc0_data;
            ++adc0_count;
         }

         if (adc1high_pga == ADC_AdaptPGA(adc1_data, adc1high_pga)) {
            adc1_sum += adc1_data;
            ++adc1_count;
         }

      } else { //low average
         if (adc0low_pga == ADC_AdaptPGA(adc0_data, adc0low_pga)) {
            adc0_sum += adc0_data;
            ++adc0_count;
         }

         if (adc1low_pga == ADC_AdaptPGA(adc1_data, adc1low_pga)) {
            adc1_sum += adc1_data;
            ++adc1_count;
         }
      }

#endif

#ifdef ALGORITHM_PEAK2PEAK

      if (GP1IN_IN0_BBA) {
         if ((adc0high_pga == ADC_AdaptPGA(adc0_data, adc0high_pga)) && (adc0_data > adc0_peak)) {
            adc0_peak = adc0_data;
         }

         if ((adc1high_pga == ADC_AdaptPGA(adc1_data, adc1high_pga)) && (adc1_data > adc1_peak)) {
            adc1_peak = adc1_data;
         }

      } else {
         if ((adc0low_pga == ADC_AdaptPGA(adc0_data, adc0low_pga)) && (adc0_data < adc0_peak)) {
            adc0_peak = adc0_data;
         }

         if ((adc1low_pga == ADC_AdaptPGA(adc1_data, adc1low_pga)) && (adc1_data < adc1_peak)) {
            adc1_peak = adc1_data;
         }
      }

#endif

      if (GP1IN_IN0_BBA) {
         adc0high_pga = ADC_AdaptPGA(adc0_data, adc0high_pga);
         adc1high_pga = ADC_AdaptPGA(adc1_data, adc1high_pga);

         AdcRng(pADI_ADC0, ADCCON_ADCREF_INTREF, (adc0high_pga << 4), ADCCON_ADCCODE_INT);
         AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, (adc1high_pga << 4), ADCCON_ADCCODE_INT);

      } else {
         adc0low_pga = ADC_AdaptPGA(adc0_data, adc0low_pga);
         adc1low_pga = ADC_AdaptPGA(adc1_data, adc1low_pga);

         AdcRng(pADI_ADC0, ADCCON_ADCREF_INTREF, (adc0low_pga << 4), ADCCON_ADCCODE_INT);
         AdcRng(pADI_ADC1, ADCCON_ADCREF_INTREF, (adc1low_pga << 4), ADCCON_ADCCODE_INT);

      }
   }
}


/****************************** Static functions *****************************/

/**
   @brief Adapt PGA

   @param data - ADC value
   @param pga - PGA value

   @return int - PGA value
**/
static int ADC_AdaptPGA(signed long data, int pga)
{
   for (auto a = 0; a < pga; ++a) {
      if (abs(data) > (0x6aaaaaa >> a)) { /* 0x6aaaaaa = 500mV ADC code */
         pga = a;
      }
   }

   for (auto a = 7; a > pga; --a) {
      if (abs(data) < (0xcaaaaaa >> a)) { /* 0xcaaaaaa = 950mV ADC code */
         pga = a;
      }
   }

   return pga;
}
