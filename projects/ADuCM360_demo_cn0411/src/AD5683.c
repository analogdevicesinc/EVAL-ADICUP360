/***************************************************************************//**
 *   @file   AD5683.c
 *   @brief  AD5683 Driver implementation file.
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
#include <AD5683.h>
#include <Communication.h>
#include <stdint.h>

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * Write commands to DAC
 *
 * @param device - device structure
 * @param cmd - command value
 * @param data - data to be written
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_write_cmd(struct ad5683_device *device, uint8_t cmd,
			 uint16_t data)
{
	uint8_t buf[3];

	buf[2] = ((data << 4) & 0xF0);
	buf[1] = (data >> 4);
	buf[0] = (data >> 12) | (cmd << 4);

	return SPI_Write(device->slave_select_id, buf, 3);
}

/**
 * Readback the contents of the input register.
 *
 * @param device - device structure
 * @param data - read data to be stored
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_readback_register(struct ad5683_device *device, uint32_t *data)
{

	int32_t ret;
	uint8_t buf[3] = {0, 0, 0};

	AD5683_write_cmd(device, AD5683_CMD_RB_IN_REG, 0x0000);
	ret = SPI_Read(device->slave_select_id, buf, 3);
	*data = (((buf[0] << 16) | (buf[1] << 8) | (buf[2])) & 0x0FFFF0) >> 4;

	return ret;
}

/**
 * Write value to DAC
 *
 * @param device - device structure
 * @param data - data to be written
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_write_dac_value(struct ad5683_device *ad5683_device,
			       uint16_t data)
{
	return AD5683_write_cmd(ad5683_device, AD5683_CMD_WR_DAC_IN_REG, data);
}

/**
 * Perform DAC Soft Reset
 *
 * @param device - device structure
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_soft_reset(struct ad5683_device *ad5683_device)
{
	return AD5683_write_cmd(ad5683_device, AD5683_CMD_WR_CTRL_REG,
				AD5683_SW_RESET(AD5683_RESET_ENABLE));
}

/**
 * Write command to DAC
 *
 * @param device - device structure
 * @param slave_select_id - chip select value corresponding to DAC
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_setup(struct ad5683_device *ad5683_device, int slave_select_id)
{
	ad5683_device->slave_select_id = slave_select_id;
	ad5683_device->dac_reg_value = 0;

	if(AD5683_soft_reset(ad5683_device) == AD5683_FAILURE)
		return AD5683_FAILURE;
	else
		return AD5683_write_cmd(ad5683_device, AD5683_CMD_WR_CTRL_REG,
					AD5683_DCEN(AD5683_DC_DISABLE)
					| AD5683_CFG_GAIN(AD5683_AMP_GAIN_1)
					| AD5683_REF_EN(AD5683_INT_REF_ON)
					| AD5683_OP_MOD(AD5683_PWR_NORMAL)
					| AD5683_SW_RESET(AD5683_RESET_DISABLE));
}

