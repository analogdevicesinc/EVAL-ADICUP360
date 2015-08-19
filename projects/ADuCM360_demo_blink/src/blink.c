/**
******************************************************************************
*   @file     blink.c
*   @brief    Source file for Blink demo.
*   @version  V0.1
*   @author   ADI
*   @date     August 2015
*
*******************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
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

#include <ADuCM360.h>

#include "blink.h"
#include "WdtLib.h"
#include "ClkLib.h"
#include "DioLib.h"
#include "Timer.h"
#include "GptLib.h"

/******************************************************************************/
/************************* Variable Definitions ******************************/
/******************************************************************************/
int		byPassTimer0 = 0;

/******************************************************************************/
/************************* Functions Definitions ******************************/
/******************************************************************************/
/**
	@brief void Blink_Sys_Init(void)
			==========System required initializations==========
**/
void Blink_Sys_Init(void)
{
	/* Disable Watchdog timer resets */
	WdtCfg(T3CON_PRE_DIV1, T3CON_IRQ_EN, T3CON_PD_DIS);

	/* Disable clock to all peripherals */
	ClkDis( CLKDIS_DISSPI0CLK| CLKDIS_DISSPI1CLK| CLKDIS_DISI2CCLK|
			CLKDIS_DISUARTCLK| CLKDIS_DISPWMCLK|
			CLKDIS_DIST1CLK| CLKDIS_DISDACCLK| CLKDIS_DISDMACLK|
			CLKDIS_DISADCCLK);

	/*Configures system clock */
	ClkCfg(CLK_CD0, CLK_HF, CLKSYSDIV_DIV2EN_DIS, CLK_UCLKCG);
	ClkSel(CLK_CD0,CLK_CD0,CLK_CD0,CLK_CD0);

#if (BLINK_USE_IRQ == YES)
	/* Initialize the general purpose timer */
	GptLd(pADI_TM0,63);                                  // Set timeout period for 0.5 seconds
	GptCfg(pADI_TM0,TCON_CLK_LFOSC,TCON_PRE_DIV256,TCON_MOD_PERIODIC|TCON_ENABLE);
#endif

}


/**
	@brief void Blink_Init(void)
			==========Application required initializations==========
**/
void Blink_Init(void)
{
	/* Set the digital outputs for the LEDs */
	DioOen(pADI_GP0, GP0OEN_OEN4_OUT|GP0OEN_OEN5_OUT);

	/* Initialize output values for LEDs */
	DioWr(pADI_GP0, GP0OUT_OUT4_LOW|GP0OUT_OUT5_HIGH);

}

#if (BLINK_USE_IRQ == NO)
/**
	@brief void Blink_Process(void)
			==========Blinking process when not using Timer IRQ==========
**/
void Blink_Process(void)
{
	/* Configure blinking interval */
	timer_sleep(BLINK_TIME*TIMER_FREQUENCY_HZ);

	/* Toggle P0.4 - LED2, P0.5 - LED3 */
	DioTgl(pADI_GP0, GPTGL_TGL4|GPTGL_TGL5);

}
#else
/**
	@brief void Blink_Interrupt_Process(void)
			==========Blinking process when using Timer IRQ==========
**/
void Blink_Interrupt_Process(void)
{
	/* Clears current Timer interrupt */
    GptClrInt(pADI_TM0,TSTA_TMOUT);

    if (!byPassTimer0) {

    	/* Toggle P0.4 - LED2, P0.5 - LED3 */
    	DioTgl(pADI_GP0, GPTGL_TGL4|GPTGL_TGL5);
    }
}
#endif

/***************************************************************************/
