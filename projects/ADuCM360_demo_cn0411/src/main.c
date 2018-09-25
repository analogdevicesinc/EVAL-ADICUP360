/***************************************************************************//**
 *   @file   main.c
 *   @brief  Main file for CN0411.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <CN0411.h>
#include <UrtLib.h>
#include <stdio.h>
#include "Timer.h"
#include <Communication.h>

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif
/* ****************************************************************************/

struct cn0411_init_params cn0411_init_params= {
	CH_GAIN_RES_20M,
	ADC_SINGLE_CONV,
	RTD_RES_100,
	{
		GAIN_RES_20,
		GAIN_RES_200,
		GAIN_RES_2K,
		GAIN_RES_20K,
		GAIN_RES_200K,
		GAIN_RES_2M,
		GAIN_RES_20M
	},
	OFFSET_RES_INIT,
	DAC_OUT_DEFAULT_VAL,
	EXC_DEFAULT_VAL,
	VR20S_DEFAULT_VAL,
	VR200S_DEFAULT_VAL,
	RDRES_DEFAULT_VAL,
	EXC_DEFAULT_VAL,
	CELL_CONST_NORMAL,
	TEMP_DEFAULT_VAL,
	VPP_DEFAULT_VAL,
	VINP_DEFAULT_VAL,
	VINN_DEFAULT_VAL,
	COND_DEFAULT_VAL,
	COMP_COND_DEFAULT_VAL,
	TDS_DEFAULT_VAL,
	{
		TEMP_COEFF_NACL,
		TDS_NACL
	}
};

/**
 * UART Interrupt Handler.
 *
 * Calls the internal interrupt handler for CN0411
 *
 * @return none.
 */
void UART_Int_Handler (void)
{
	CN0411_interrupt();
}

/**
 * Main Function
 *
 * @param argc - Arguments count
 * @param argv - Arguments value
 * @return 0 in case of success, negative error code otherwise.
 */
int main(int argc, char *argv[])
{
	struct cn0411_device cn0411_dev;
	uint32_t ret;

	timer_start();              /* Start the System Tick Timer. */
	timer_start_us();

	/* Initialize UART */
	UART_Init (B115200, COMLCR_WLS_8BITS);

	/* Initialize SPI */
	ret = SPI_Init();
	if (ret == CN0411_FAILURE)
		return ret;

	/* Initialize CN0411 */
	ret = CN0411_init(&cn0411_dev, cn0411_init_params);

	if (ret == CN0411_FAILURE) {
		printf("CN0411 Initialization error!\n");
		return ret;
	}
	while (1)
		CN0411_cmd_process(&cn0411_dev);    /* Command line process */

	return ret;
}

