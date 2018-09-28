/*!
 *****************************************************************************
 * @file:    GptLib.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

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

/**
 *****************************************************************************
   @addtogroup gpt
   @{
   @file     GptLib.c
   @brief Set of Timer peripheral functions.
   - Example:

   @version  V0.5
   @author   ADI
   @date     October 2015

   @par Revision History:
   - V0.1, May 2012: initial version.
   - V0.2, February 2013:   Fixed GptBsy().
   - V0.3, April 2013: fixed capture event list in GptCapSrc()
   - V0.4, October 2015: Coding style cleanup - no functional changes.
   - V0.5, October 2015: Use Standard Integer Types, prefer unsigned types, add include and C guards.

**/

#include "GptLib.h"

/**
   @brief uint32_t GptCfg(ADI_TIMER_TypeDef *pTMR, uint32_t iClkSrc, uint32_t iScale, uint32_t iMode)
         ==========Configures timer GPTx if not busy.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
   @param iClkSrc :{TCON_CLK_UCLK,TCON_CLK_PCLK,TCON_CLK_LFOSC,TCON_CLK_LFXTAL}
      - TxCON.5,6
      - TCON_CLK_UCLK, the timer is clocked by system clock.
      - TCON_CLK_PCLK, the timer is clocked by peripheral clock.
      - TCON_CLK_LFOSC, the timer is clocked by 32kHz clock.
      - TCON_CLK_LFXTAL, the timer is clocked by external 32kHz crystal.
   @param iScale :{TCON_PRE_DIV1,TCON_PRE_DIV16,TCON_PRE_DIV256,TCON_PRE_DIV32768}
      - TxCON.0,1
      - TCON_PRE_DIV1 for prescale of 1 (or 4 if UCLK chosen by iSrc).
      - TCON_PRE_DIV16 for prescale of 16.
      - TCON_PRE_DIV256 for prescale of 256.
      - TCON_PRE_DIV32768 for prescale of 32768.
   @param iMode :{TCON_MOD_PERIODIC|TCON_UP|TCON_RLD|TCON_ENABLE|TCON_EVENTEN}
      - TxCON.2-4,7,12
      - TCON_MOD_PERIODIC = 1  for the timer periodic mode. TCON_MOD_FREERUN or 0 by default.
      - TCON_UP = 1 to count down. 0 to count up. .
      - TCON_RLD = TCON_RLD_DIS or TCON_RLD_EN for reload on everflow. TCON_RLD_DIS by default.
      - TCON_ENABLE = TCON_ENABLE_DIS or TCON_ENABLE_EN to enable timer. TCON_ENABLE_DIS by default.
      - TCON_EVENT = TCON_EVENT_DIS or TCON_EVENTEN_EN to enable capture mode. TCON_EVENT_DIS by default.
   @return 0 if timer interface busy or 1 if successfull.

**/

uint32_t GptCfg(ADI_TIMER_TypeDef *pTMR, uint32_t iClkSrc, uint32_t iScale,
		uint32_t iMode)
{
	uint32_t i1;

	if(pTMR->STA & TSTA_CON) {
		return 0;
	}

	i1 = pTMR->CON & TCON_EVENT_MSK; // to keep the selected event
	i1 |= iClkSrc;
	i1 |= iScale;
	i1 |= iMode;
	pTMR->CON = i1;
	return 1;
}

/**
   @brief uint32_t GptLd(ADI_TIMER_TypeDef *pTMR, uint32_t iTLd);
         ==========Sets timer reload value.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
   @param iTLd :{0-65535}
      - Sets reload value TxLD to iTLd.
   @return 1.
**/

uint32_t GptLd(ADI_TIMER_TypeDef *pTMR, uint32_t iTLd)
{
	pTMR->LD = iTLd;
	return 1;
}


/**
   @brief uint32_t GptVal(ADI_TIMER_TypeDef *pTMR);
         ==========Reads timer value.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
   @return timer value TxVAL.
**/

uint32_t GptVal(ADI_TIMER_TypeDef *pTMR)
{
	return pTMR->VAL;
}

/**
   @brief uint32_t GptCapRd(ADI_TIMER_TypeDef *pTMR);
         ==========Reads capture value. Allows capture of a new value.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
   @return capture value TxCAP.
**/

uint32_t GptCapRd(ADI_TIMER_TypeDef *pTMR)
{
	return pTMR->CAP;
}

/**
   @brief uint32_t GptCapSrc(ADI_TIMER_TypeDef *pTMR, uint32_t iTCapSrc);
         ==========Sets capture source.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
   @param iTCapSrc :{T0CON_EVENT_T2, T0CON_EVENT_EXT0, T0CON_EVENT_EXT1, T0CON_EVENT_EXT2,
                        T0CON_EVENT_EXT3, T0CON_EVENT_EXT4, T0CON_EVENT_EXT5, T0CON_EVENT_EXT6,
                        T0CON_EVENT_EXT7, T0CON_EVENT_T3, T0CON_EVENT_T1,
                        T0CON_EVENT_ADC0, T0CON_EVENT_ADC1, T0CON_EVENT_STEP, T0CON_EVENT_FEE,
         T1CON_EVENT_COM, T1CON_EVENT_T0, T1CON_EVENT_SPI0, T1CON_EVENT_SPI1,
         T1CON_EVENT_I2CS, T1CON_EVENT_I2CM, T1CON_EVENT_DMAERR, T1CON_EVENT_DMADONE,
                        T1CON_EVENT_EXT1, T1CON_EVENT_EXT2, T1CON_EVENT_EXT3, T1CON_EVENT_PWMTRIP,
                        T1CON_EVENT_PWM0, T1CON_EVENT_PWM1, T1CON_EVENT_PWM2}
      - TxCON.8-11
                - for timer 0 capture event, select one of the following event:
         T0CON_EVENT_T2, T0CON_EVENT_EXT0, T0CON_EVENT_EXT1, T0CON_EVENT_EXT2,
                        T0CON_EVENT_EXT3, T0CON_EVENT_EXT4, T0CON_EVENT_EXT5, T0CON_EVENT_EXT6,
                        T0CON_EVENT_EXT7, T0CON_EVENT_T3, T0CON_EVENT_T1,
                        T0CON_EVENT_ADC0, T0CON_EVENT_ADC1, T0CON_EVENT_STEP, T0CON_EVENT_FEE
                - for timer 1  capture event, select one of the following event:
         T1CON_EVENT_COM, T1CON_EVENT_T0, T1CON_EVENT_SPI0, T1CON_EVENT_SPI1,
         T1CON_EVENT_I2CS, T1CON_EVENT_I2CM, T1CON_EVENT_DMAERR, T1CON_EVENT_DMADONE,
                        T1CON_EVENT_EXT1, T1CON_EVENT_EXT2, T1CON_EVENT_EXT3, T1CON_EVENT_PWMTRIP,
                        T1CON_EVENT_PWM0, T1CON_EVENT_PWM1, T1CON_EVENT_PWM2
   @return 1.
**/

uint32_t GptCapSrc(ADI_TIMER_TypeDef *pTMR, uint32_t iTCapSrc)
{
	uint32_t i1;

	if(pTMR->STA & TSTA_CON) {
		return 0;
	}

	i1 = pTMR->CON & (~TCON_EVENT_MSK);
	i1 |= iTCapSrc;
	pTMR->CON = i1;
	return 1;
}


/**
   @brief uint32_t GptSta(ADI_TIMER_TypeDef *pTMR);
         ==========Reads timer status register.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
   @return TxSTA.
**/

uint32_t GptSta(ADI_TIMER_TypeDef *pTMR)
{
	return pTMR->STA;
}


/**
   @brief uint32_t GptClrInt(ADI_TIMER_TypeDef *pTMR, uint32_t iSource);
         ==========clears current Timer interrupt by writing to TxCLRI.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
   @param iSource :{TSTA_TMOUT,TSTA_CAP}
      - TSTA_TMOUT for time out.
      - TSTA_CAP for capture event.
   @return 1.
**/

uint32_t GptClrInt(ADI_TIMER_TypeDef *pTMR, uint32_t iSource)
{
	pTMR->CLRI = iSource;
	return 1;
}

/**
   @brief uint32_t GptBsy(ADI_TIMER_TypeDef *pTMR);
         ==========Checks the busy bit.
   @param pTMR :{pADI_TM0,pADI_TM1}
      - pADI_TM0 for timer 0.
      - pADI_TM1 for timer 1.
@return busy bit: 0 is not busy, 1 is busy.
**/
uint32_t GptBsy(ADI_TIMER_TypeDef *pTMR)
{
	if (pTMR == pADI_TM0) {
		return T0STA_CON_BBA;

	} else {
		return T1STA_CON_BBA;
	}
}

/**@}*/
