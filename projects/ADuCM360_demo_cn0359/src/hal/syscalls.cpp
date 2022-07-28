/***************************************************************************//**
 *   @file   syscalls.cpp
 *   @brief  Implementation of syscalls.cpp
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

/* Support files for GNU libc.  Files in the system namespace go here.
 Files in the C namespace (ie those that do not start with an
 underscore) go in .c.  */

#include <unistd.h>
#include <cassert>
#include <cstring>
#include <errno.h>
#include <hal/drivers/flash.h>
#include <hal/drivers/lcd.h>
#include <hal/drivers/uart.h>
#include <hal/drivers/dac.h>
#include <hal/drivers/adc.h>
#include <hal/drivers/pwm.h>
#include <hal/drivers/ad8253.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* following is copied from libc/stdio/local.h to check std streams */
extern void __sinit(struct _reent *);

#define CHECK_INIT(ptr)\
  do\
    {\
      if ((ptr) && !(ptr)->__sdidinit)\
	__sinit (ptr);\
    }\
  while (0)

#define UART_HANDLE (0x20)
#define LCD_HANDLE (0x21)
#define FLASH_HANDLE (0x22)
#define DAC_HANDLE (0x23)
#define ADC_HANDLE (0x24)
#define PWM_HANDLE (0x25)
#define AD8253_HANDLE (0x26)

ssize_t _read(int fd, void *buf, size_t count)
{
	switch (fd) {
	case STDIN_FILENO:
	case UART_HANDLE:
		return uart_read(buf, count);
		break;
	case FLASH_HANDLE:
		return flash_read(buf, count);
		break;
	case DAC_HANDLE:
		return dac_read(buf, count);
		break;
	case ADC_HANDLE:
		return adc_read(buf, count);
		break;
	case PWM_HANDLE:
		return pwm_read(buf, count);
		break;
	case AD8253_HANDLE:
		return ad8253_read(buf, count);
	case LCD_HANDLE:
	case STDOUT_FILENO:
	case STDERR_FILENO:
	default:
		break;
	}
	errno = EBADF;
	return -1;
}

off_t _lseek(int fd, off_t offset, int whence)
{
	switch (fd) {
	case LCD_HANDLE:
		return lcd_tty_lseek(offset, whence);
		break;
	case FLASH_HANDLE:
		return flash_lseek(offset, whence);
		break;
	case DAC_HANDLE:
		return dac_lseek(offset, whence);
		break;
	case ADC_HANDLE:
		return adc_lseek(offset, whence);
		break;
	case PWM_HANDLE:
		return pwm_lseek(offset, whence);
		break;
	case AD8253_HANDLE:
		return ad8253_lseek(offset, whence);
		break;
	case UART_HANDLE:
	case STDIN_FILENO:
	case STDOUT_FILENO:
	case STDERR_FILENO:
		errno = ESPIPE;
		return -1;
		break;
	default:
		break;
	}
	errno = EBADF;
	return -1;
}

ssize_t _write(int fd, const void *buf, size_t count)
{
	switch (fd) {
	case LCD_HANDLE:
		return lcd_tty_write(buf, count);
		break;
	case FLASH_HANDLE:
		return flash_write(buf, count);
		break;
	case DAC_HANDLE:
		return dac_write(buf, count);
		break;
	case ADC_HANDLE:
		return adc_write(buf, count);
		break;
	case PWM_HANDLE:
		return pwm_write(buf, count);
		break;
	case AD8253_HANDLE:
		return ad8253_write(buf, count);
		break;
#ifdef DEBUG
	case STDERR_FILENO:
#endif
	case STDOUT_FILENO:
	case UART_HANDLE:
		return uart_write(buf, count);
		break;
	default:
		break;
	}

	errno = EBADF;
	return -1;
}

int _open(const char *pathname, int flags)
{
	if (strcmp(pathname, "lcd") == 0) {
		lcd_open();
		return LCD_HANDLE;
	}

	if (strcmp(pathname, "uart") == 0) {
		uart_open();
		return UART_HANDLE;
	}

	if (strcmp(pathname, "flash") == 0) {
		flash_open();
		return FLASH_HANDLE;
	}

	if (strcmp(pathname, "dac") == 0) {
		dac_open();
		return DAC_HANDLE;
	}

	if (strcmp(pathname, "adc") == 0) {
		adc_open();
		return ADC_HANDLE;
	}

	if (strcmp(pathname, "pwm") == 0) {
		pwm_open();
		return PWM_HANDLE;
	}

	if (strcmp(pathname, "ad8253") == 0) {
		ad8253_open();
		return AD8253_HANDLE;
	}

	errno = ENOENT;
	return -1;
}

#ifdef __cplusplus
}
#endif
