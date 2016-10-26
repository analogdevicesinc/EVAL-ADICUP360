/******************************************************************************
*   @file     ADC.c
*   @brief    Source file for ADC handling
*   @version  V0.1
*   @author   ADI
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
*******************************************************************************/


/***************************** Include Files **********************************/

#include <AdcLib.h>
#include <IexcLib.h>


#include "ADC.h"

/********************************* Definitions ********************************/
volatile int32_t adc0_data, adc1_data;

//ADC0 - RTD
void ADC0INIT(void)
{

    AdcRng(pADI_ADC0,ADCCON_ADCREF_EXTREF,ADCMDE_PGA_G1,ADCCON_ADCCODE_INT); // Internal Reference selected, PGA gain of 1, bipolar mode
    AdcBuf(pADI_ADC0,ADCCFG_EXTBUF_OFF,ADCCON_BUFBYPN|ADCCON_BUFBYPP|ADCCON_BUFPOWP|ADCCON_BUFPOWN);
    AdcFlt(pADI_ADC0,51,0x000,FLT_NORMAL|ADCFLT_CHOP);                                  // Select 50Hz sampling rate=> rate = 125000/[(SF+1)*16*(3+AF)+3]; SF = 51; AF = 0
    AdcMski(pADI_ADC0,ADCMSKI_RDY,1);                                        // Enable ADC0 interrupt
    AdcGo(pADI_ADC0,ADCMDE_ADCMD_IDLE);                                      // Enable ADC0 in idle mode

}

//ADC1 -Thermocouple
void ADC1INIT(void)
{

     AdcBias(pADI_ADC1,ADCCFG_PINSEL_AIN11,ADC_BIAS_X1,0);              //vbias ain11 buffers on
     AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G8,ADCCON_ADCCODE_INT); // Internal Reference selected, PGA gain of 16, bipolar mode
     AdcBuf(pADI_ADC1,ADCCFG_EXTBUF_OFF,ADCCON_BUFBYPN|ADCCON_BUFBYPP|ADCCON_BUFPOWP|ADCCON_BUFPOWN);                       // ADC input buffer on. Leave External reference buffer on for RTD measurement
     AdcFlt(pADI_ADC0,51,0x000,FLT_NORMAL|ADCFLT_CHOP);
     AdcMski(pADI_ADC1,ADCMSKI_RDY,1);                                        // Enable ADC0 interrupt
     AdcGo(pADI_ADC1,ADCMDE_ADCMD_IDLE);

}


void IEXCINIT(void)
{

   IexcDat(IEXCDAT_IDAT_300uA,IDAT0En);
}
