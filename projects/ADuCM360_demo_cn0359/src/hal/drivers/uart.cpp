/***************************************************************************//**
 *   @file   uart.cpp
 *   @brief  Implementation of uart.cpp
 *   @author
 *******************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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
#include <cassert>
#include <cmath>
#include <cstring>
#include <hal/drivers/uart.h>
#include <applications/message.h>
#include <applications/uart_exec.h>
#include <hal/drivers/flash.h>
#include <hal/devices.h>
#include <cstdio>

static char tx_fifo[512];
static volatile int tx_fifo_front = 0, tx_fifo_back = 0;

static char rx_pingpong[2][128];
static volatile int rd_count[2] = {0, 0};
static volatile int rx_length[2] = {0, 0};
static volatile char * volatile p_rx_buf = rx_pingpong[0];

#define SIZE(a) (sizeof(a)/sizeof(a[0]))

float uart_baud(float baud)
{
	/*
	 * k = 1000, M = 1000k
	 *
	 *                     UCLK/DIV                    16M/1                    500k                 1024M
	 * baud rate = ──────────────────────── = ──────────────────────── = ─────────────────── = ───────────────────
	 *              2*(M+N/2048)*16*COMDIV     2*16*(M+N/2048)*COMDIV     (M+N/2048)*COMDIV     (2048*M+N)*COMDIV
	 *
	 * set F = (2048*M+N), M = 1~3, N = 0~2047, F = 2048~8191
	 *
	 *               1024M       1024M
	 * baud rate = ────────── = ───────
	 *              F*COMDIV      UDIV
	 */

	assert(baud >= 2 && baud <= 500000);

	int high_udiv = ceil(1.024e9f / baud), low_udiv = floor(1.024e9f / baud),
	    udiv = lround(1.024e9f / baud);

	int min_comdiv = ceil(udiv / 8191.f), max_comdiv = floor(udiv / 2048.f);

	while (true) {
		for (int comdiv = min_comdiv; comdiv <= max_comdiv; ++comdiv) {
			if (udiv % comdiv == 0) {
				int DIVM, DIVN;

				DIVM = floor((udiv / comdiv) / 2048);
				DIVN = (udiv / comdiv) % 2048;

				pADI_UART->COMDIV = comdiv;

				pADI_UART->COMFBR = COMFBR_ENABLE_EN | (DIVM << 11) | DIVN;

				return (1.024e9f / ((2048 * DIVM + DIVN) * comdiv));
			}
		}

//		no matched udiv, search closest value
		if (udiv == high_udiv) {
//			max udiv = (2048 * 3 + 2047) * 65535
			if (high_udiv < 536797185) {
				++high_udiv;
			}

			udiv = low_udiv;
		} else {
//			min udiv = (2048 * 1 + 0) * 1
			if (low_udiv > 2048) {
				--low_udiv;
			}

			udiv = high_udiv;
		}

		min_comdiv = ceil(udiv / 8191.f);
		max_comdiv = floor(udiv / 2048.f);
	}
}

void uart_open(void)
{
	CLKDIS_DISUARTCLK_BBA = false;
	pADI_CLKCTL->CLKCON1 = (pADI_CLKCTL->CLKCON1 & ~CLKCON1_UARTCD_MSK) |
			       CLKCON1_UARTCD_DIV1;

	//	schematic
	pADI_GP0->GPCON = (pADI_GP0->GPCON & ~GP0CON_CON6_MSK) | GP0CON_CON6_UARTRXD;
	pADI_GP0->GPCON = (pADI_GP0->GPCON & ~GP0CON_CON7_MSK) | GP0CON_CON7_UARTTXD;

	GP0OCE_OCE6_BBA = false;
	GP0OCE_OCE7_BBA = false;

	GP0OEN_OEN6_BBA = false;
	GP0OEN_OEN7_BBA = true;

	GP0PUL_PUL6_BBA = true;
	GP0PUL_PUL7_BBA = true;

	pADI_UART->COMCON = COMCON_DISABLE_DIS;

	pADI_UART->COMLCR = COMLCR_BRK_DIS | COMLCR_SP_DIS | COMLCR_EPS_DIS |
			    COMLCR_PEN_DIS | COMLCR_STOP_EN
			    | COMLCR_WLS_8BITS;

	pADI_UART->COMMCR = COMMCR_LOOPBACK_DIS | COMMCR_OUT1_DIS | COMMCR_OUT2_DIS |
			    COMMCR_RTS_DIS | COMMCR_DTR_DIS;

	flash_file * p_flash_file;
	decltype(flash_file::baud_rate) baud_rate;

	fseek(p_flash, (int) (&p_flash_file->baud_rate) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&baud_rate, sizeof(flash_file::baud_rate), 1, p_flash);

	uart_baud(baud_rate);

	pADI_UART->COMIEN = COMIEN_EDMAR_DIS | COMIEN_EDMAT_DIS | COMIEN_EDSSI_DIS |
			    COMIEN_ELSI_DIS | COMIEN_ETBEI_EN
			    | COMIEN_ERBFI_EN;

	NVIC_SetPriority(UART_IRQn, NVIC_EncodePriority(6, 1, 2));

	NVIC_EnableIRQ(UART_IRQn);
}

ssize_t uart_read(void *buf, size_t count)
{
	if (p_rx_buf == rx_pingpong[0]) {
		if (rd_count[1] + count > rx_length[1]) {
			count = rx_length[1] - rd_count[1];
		}

		memcpy(buf, &rx_pingpong[1][rd_count[1]], count);

		rd_count[1] += count;
	} else {
		if (rd_count[0] + count > rx_length[0]) {
			count = rx_length[0] - rd_count[0];
		}

		memcpy(buf, &rx_pingpong[0][rd_count[0]], count);

		rd_count[0] += count;
	}

	return count;
}

ssize_t uart_write(const void *buf, size_t count)
{
	for (int i = 0; i < count; ++i) {
		tx_fifo[tx_fifo_back] = ((char*) (buf))[i];

		if (tx_fifo_back == SIZE(tx_fifo) - 1) {
			tx_fifo_back = 0;
		} else {
			++tx_fifo_back;
		}

		assert(tx_fifo_back != tx_fifo_front); // overflow
	}

	if (COMLSR_TEMT_BBA) {
		char c = tx_fifo[tx_fifo_front];

		if (tx_fifo_front == SIZE(tx_fifo) - 1) {
			tx_fifo_front = 0;
		} else {
			++tx_fifo_front;
		}

		pADI_UART->COMTX = c;
	}

	return count;
}

static int tx_empty(int argc, char *argv[])
{
	if (!COMLSR_TEMT_BBA) {
		app msg;
		msg.argc = 0;
		msg.fun = tx_empty;
		msg.argv = new char*;
		post_message(msg);
	}
}

#ifdef __cplusplus
extern "C"
{
#endif
void UART_Int_Handler(void)
{
	if ((pADI_UART->COMIIR & COMIIR_STA_TXBUFEMPTY) == COMIIR_STA_TXBUFEMPTY) {
		if (tx_fifo_front != tx_fifo_back) {
			pADI_UART->COMTX = tx_fifo[tx_fifo_front];

			if (tx_fifo_front == SIZE(tx_fifo) - 1) {
				tx_fifo_front = 0;
			} else {
				++tx_fifo_front;
			}
		} else {
			app msg;
			msg.argc = 0;
			msg.fun = tx_empty;
			ts_post_message(msg);
		}
	}

	if ((pADI_UART->COMIIR & COMIIR_STA_RXBUFFULL) == COMIIR_STA_RXBUFFULL) {
		if (p_rx_buf == rx_pingpong[0]) {
			if (rx_length[0] < SIZE(rx_pingpong[0])) {
				p_rx_buf[rx_length[0]++] = pADI_UART->COMRX;
			}
		} else {
			if (rx_length[1] < SIZE(rx_pingpong[1])) {
				p_rx_buf[rx_length[1]++] = pADI_UART->COMRX;
			}
		}

		if (pADI_UART->COMRX == '\n') {
			if (p_rx_buf == rx_pingpong[0]) {
				p_rx_buf = rx_pingpong[1];
				rx_length[1] = 0;
				rd_count[1] = 0;
			} else {
				p_rx_buf = rx_pingpong[0];
				rx_length[0] = 0;
				rd_count[0] = 0;
			}

			app msg;
			msg.argc = 0;
			msg.fun = rx_line;
			ts_post_message(msg);
		}
	}
}
#ifdef __cplusplus
}
#endif
