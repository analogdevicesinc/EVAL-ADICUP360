/***************************************************************************//**
 *   @file   flash.cpp
 *   @brief  Implementation of flash.cpp
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
#include <cstring>
#include <cassert>
#include <errno.h>
#include <hal/drivers/flash.h>

static int position = 0;

//flash_disk exclusive in his own flash sector
static const struct flash_file flash_disk __attribute__ ((aligned (512),
		section (".flash_disk")));

static const char * const p_flash_disk = (const char *) (&flash_disk);

ssize_t flash_write(const void *buf, size_t count)
{
	if (position >= sizeof(flash_file)) {
		errno = ENOSPC;
		return -1;
	}

	int start_page = position >> 9;

	int end_page = ((position + count) < sizeof(flash_file) ?
			(position + count - 1) : (sizeof(flash_file) - 1)) >> 9;

	int remain_bytes = count;
	const char *remain_buf;
	remain_buf = (const char *) buf;

	for (int i = start_page; i <= end_page; ++i) {
		char buffer[512] __attribute__ ((aligned (sizeof(unsigned long))));

		memcpy(buffer, &p_flash_disk[i << 9], 512);

		pADI_FEE->FEEADR0H = (((int) (&p_flash_disk[i << 9]) >> 16) & 0x1);
		pADI_FEE->FEEADR0L = ((int) (&p_flash_disk[i << 9]) & 0xfe00);

		pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY1;
		pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY2;

		pADI_FEE->FEECMD = FEECMD_CMD_ERASEPAGE; // erase
		while (FEESTA_CMDBUSY_BBA)
			;

		int write_count = (position & 0x1ff) + remain_bytes > 512 ? 512 -
				  (position & 0x1ff) : remain_bytes;

		memcpy(&buffer[position & 0x1ff], remain_buf, write_count);

		FEECON0_WREN_BBA = true;

		unsigned long *p_src, *p_dst;
		p_src = (unsigned long *) buffer;
		p_dst = (unsigned long *) (&p_flash_disk[i << 9]);

		for (int i = 0; i < 128; ++i) {
			while (FEESTA_WRBUSY_BBA)
				;
			p_dst[i] = p_src[i];
		}

		FEECON0_WREN_BBA = false;

		position += write_count;
		remain_bytes -= write_count;
		remain_buf += write_count;
	}

	return (count - remain_bytes);
}

ssize_t flash_read(void *buf, size_t count)
{
	if ((position + count) > sizeof(flash_file)) {
		count = sizeof(flash_file) - position;
	}

	memcpy(buf, &p_flash_disk[position], count);

	position += count;

	return count;
}

off_t flash_lseek(off_t offset, int whence)
{
	switch (whence) {
	case SEEK_SET:
		if (offset > sizeof(flash_file) || offset < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = offset;
		}

		break;
	case SEEK_CUR:
		if ((position + offset) > sizeof(flash_file) || (position + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position += offset;
		}
		break;
	case SEEK_END:
		if (offset > 0 || sizeof(flash_file) + offset < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = sizeof(flash_file) + offset;
		}
		break;
	default:
		errno = ENXIO;
		return -1;
		break;
	}

	return position;
}

void flash_open(void)
{

}
