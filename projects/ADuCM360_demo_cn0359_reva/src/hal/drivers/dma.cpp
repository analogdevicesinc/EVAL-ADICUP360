/***************************************************************************//**
 *   @file   dma.cpp
 *   @brief  Implementation of dma.cpp
 *   @author
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
 ******************************************************************************/
#include <ADuCM360.h>
#include <hal/drivers/dma.h>
#include <hal/drivers/lcd.h>

static struct dma_channel_control dma_control[32] __attribute__((aligned(sizeof(
			dma_channel_control)*32))); //512 bytes

void initial_dma(void)
{
	CLKDIS_DISDMACLK_BBA = false;

	pADI_DMA->DMAPDBPTR = (uint32_t) dma_control;
	pADI_DMA->DMACFG = DMACFG_ENABLE_EN;

	dma_channel_control channel_control;

	channel_control.src_end = (unsigned int) (&framebuffer_memory[3][(132 / 4) -
				  1]);
	channel_control.src_end += (sizeof(int) - 1);

	channel_control.dst_end = (unsigned long) &pADI_SPI1->SPITX;

	channel_control.control_data.dst_inc = 0x3; //no increment
	channel_control.control_data.src_inc = 0x0; //byte increment
	channel_control.control_data.src_size = 0x0; //byte
	channel_control.control_data.r_power = 0x0; //rearbitrate in each byte
	channel_control.control_data.n_minus_1 = ((4 * 132) - 1); //half of screen
	channel_control.control_data.cycle_ctrl = 0x0; //stop

	dma_control[0] = channel_control; //primary

	channel_control.src_end = (unsigned int) (&framebuffer_memory[7][(132 / 4) -
				  1]);
	channel_control.src_end += (sizeof(int) - 1);

	dma_control[16] = channel_control; //alternate

	DMARMSKCLR_SPI1TX_BBA = true;

	DMAALTCLR_SPI1TX_BBA = true;

	DMAPRISET_SPI1TX_BBA = true;

	DMAERRCLR_ERROR_BBA = true;
}

int enable_dma_spi1tx(int argc, char *argv[])
{
	//pirmary
	dma_control[0].control_data.n_minus_1 = ((4 * 132) - 1); //half of screen
	dma_control[0].control_data.cycle_ctrl = 0x3; //ping pong
	//alternate
	dma_control[16].control_data.n_minus_1 = ((4 * 132) - 1); //half of screen
	dma_control[16].control_data.cycle_ctrl = 0x3; //ping pong

	DMAENSET_SPI1TX_BBA = true;

	return 0;
}
