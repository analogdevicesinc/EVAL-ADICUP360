/***************************************************************************//**
 *   @file   CN0411.c
 *   @brief  CN0411 source file
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

#include "AD7124_regs.h"
#include "CN0411.h"
#include <Communication.h>
#include <UrtLib.h>
#include <PwmLib.h>
#include "Timer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include "DioLib.h"
#include "math.h"

/******************************************************************************/
/*************************** Variable Definitions *****************************/
/******************************************************************************/

/* Available commands */
char *cmd_commands[] = {
	"help",
	"syscal",
	"refres",
	"convmod",
	"autoset",
	"setdac",
	"gainres",
	"rtdval",
	"pwmfreq",
	"cellconst",
	"solution",
	"temp",
	"vinput",
	"readdac",
	"rdr20s",
	"rdr200s",
	"rdres",
	"cond",
	"tds"
};

/* Functions for available commands */
cmd_func cmd_fun[] = {
	CN0411_cmd_help,
	CN0411_cmd_sys_calib,
	CN0411_cmd_off_res,
	CN0411_cmd_conv_mode,
	CN0411_cmd_autoset,
	CN0411_cmd_set_dac,
	CN0411_cmd_gain_res,
	CN0411_cmd_rtd_val,
	CN0411_cmd_pwm_freq,
	CN0411_cmd_cell_const,
	CN0411_cmd_solution,
	CN0411_cmd_temp,
	CN0411_cmd_vinput,
	CN0411_cmd_read_dac,
	CN0411_cmd_read_20s,
	CN0411_cmd_read_200s,
	CN0411_cmd_rdres,
	CN0411_cmd_cond,
	CN0411_cmd_tds,
	NULL
};

/******************************************************************************/
/************************* Function Definitions *******************************/
/******************************************************************************/

/**
 * CN0411 set DAC output value
 *
 * @param cn0411_dev - The device structure.
 * @param output_val - voltage value to be written on DAC
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_DAC_set_value(struct cn0411_device *cn0411_dev, float output_val)
{
	cn0411_dev->ad5683_dev.dac_reg_value = (uint32_t)(output_val * DAC_FS_VAL /
					       VREF);

	return AD5683_write_dac_value(&cn0411_dev->ad5683_dev,
				      cn0411_dev->ad5683_dev.dac_reg_value);
}

/**
 * CN0411 ADC Operation Mode function
 *
 * @param cn0411_dev - The device structure.
 * @param mode - operation mode of the ADC
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_operation_mode (struct cn0411_device * cn0411_dev,
				   enum op_mode mode)
{
	int32_t mask;

	mask = (AD7124_ADC_CTRL_REG_REF_EN |
		AD7124_ADC_CTRL_REG_DATA_STATUS |
		AD7124_ADC_CTRL_REG_POWER_MODE(LOW_POWER) |
		AD7124_ADC_CTRL_REG_MODE(mode) |
		AD7124_ADC_CTRL_REG_CLK_SEL(INTERNAL_CLK1));

	cn0411_dev->ad7124_dev.regs[AD7124_ADC_Control].value = mask;

	return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
				    cn0411_dev->ad7124_dev.regs[AD7124_ADC_Control]);
}

/**
 * CN0411 ADC Setup 0 configuration function
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_setup (struct cn0411_device *cn0411_dev)
{
	int32_t mask;

	mask = (AD7124_CFG_REG_BURNOUT(0) | AD7124_CFG_REG_REF_BUFP |
		AD7124_CFG_REG_REF_BUFM | AD7124_CFG_REG_AIN_BUFP |
		AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(0) |
		AD7124_CFG_REG_PGA(0));

	cn0411_dev->ad7124_dev.regs[AD7124_Config_0].value = mask;

	if(AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
				cn0411_dev->ad7124_dev.regs[AD7124_Config_0]) == CN0411_FAILURE)
		return CN0411_FAILURE;

	mask = (AD7124_CFG_REG_BURNOUT(0) |
		AD7124_CFG_REG_REF_BUFP |
		AD7124_CFG_REG_REF_BUFM |
		AD7124_CFG_REG_AIN_BUFP |
		AD7124_CFG_REG_AINN_BUFM |
		AD7124_CFG_REG_REF_SEL(2) |
		AD7124_CFG_REG_PGA(0));

	cn0411_dev->ad7124_dev.regs[AD7124_Config_1].value = mask;

	if(AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
				cn0411_dev->ad7124_dev.regs[AD7124_Config_1]) == CN0411_FAILURE)
		return CN0411_FAILURE;

	return CN0411_SUCCESS;
}

/**
 * CN0411 ADC Channels configuration function
 *
 * @param cn0411_dev - The device structure.
 * @param ch_en - enable/disable ADC channel
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_set_ch (struct cn0411_device *cn0411_dev, uint8_t channel,
			   uint8_t ch_en )
{
	int32_t mask;

	switch(channel) {
	case ADC_CH0:
		mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(0) |
			AD7124_CH_MAP_REG_AINP(1) | AD7124_CH_MAP_REG_AINM(6));
		cn0411_dev->ad7124_dev.regs[AD7124_Channel_0].value = mask;
		break;
	case ADC_CH1:
		mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
			AD7124_CH_MAP_REG_AINP(7) | AD7124_CH_MAP_REG_AINM(17));
		cn0411_dev->ad7124_dev.regs[AD7124_Channel_1].value = mask;
		break;
	case ADC_CH2:
		mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
			AD7124_CH_MAP_REG_AINP(8) | AD7124_CH_MAP_REG_AINM(17));
		cn0411_dev->ad7124_dev.regs[AD7124_Channel_2].value = mask;
		break;
	case ADC_CH3:
		mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
			AD7124_CH_MAP_REG_AINP(9) | AD7124_CH_MAP_REG_AINM(17));
		cn0411_dev->ad7124_dev.regs[AD7124_Channel_3].value = mask;
		break;
	case ADC_CH4:
		mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
			AD7124_CH_MAP_REG_AINP(10) | AD7124_CH_MAP_REG_AINM(17));
		cn0411_dev->ad7124_dev.regs[AD7124_Channel_4].value = mask;
		break;
	case ADC_CH5:
		mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
			AD7124_CH_MAP_REG_AINP(11) | AD7124_CH_MAP_REG_AINM(17));
		cn0411_dev->ad7124_dev.regs[AD7124_Channel_5].value = mask;
		break;
	default:
		return CN0411_FAILURE;
	}

	return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
				    cn0411_dev->ad7124_dev.regs[AD7124_Channel_0 + channel]);
}

/**
 * CN0411 ADC IO Control 1 configuration function
 *
 * @param cn0411_dev - The device structure.
 * @param ch_gain - select gain channel to be used.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_set_io1 (struct cn0411_device *cn0411_dev, uint8_t ch_gain)
{
	int32_t mask;

	mask = (((ch_gain + 1) << 20) |
		(AD7124_8_IO_CTRL1_REG_GPIO_CTRL3 |
		 AD7124_8_IO_CTRL1_REG_GPIO_CTRL2 |
		 AD7124_8_IO_CTRL1_REG_GPIO_CTRL1) |
		AD7124_IO_CTRL1_REG_IOUT0(3) |
		AD7124_IO_CTRL1_REG_IOUT_CH0(0));

	cn0411_dev->ad7124_dev.regs[AD7124_IOCon1].value = mask;

	return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
				    cn0411_dev->ad7124_dev.regs[AD7124_IOCon1]);
}

/**
 * CN0411 ADC IO Control 2 configuration function
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
**/
int32_t CN0411_ADC_set_io2 (struct cn0411_device *cn0411_dev)
{
	/* All VBias off */
	cn0411_dev->ad7124_dev.regs[AD7124_IOCon2].value = 0x0000;

	return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
				    cn0411_dev->ad7124_dev.regs[AD7124_IOCon2]);
}

/**
 * CN0411 ADC conversion mode initialization
 *
 * @param cn0411_dev - The device structure.
 * @param conv_mod - set conversion mode
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_conv_init(struct cn0411_device *cn0411_dev, uint8_t conv_mod)
{
	switch(conv_mod) {
	case ADC_SINGLE_CONV:
		for (uint8_t ch = ADC_CH0; ch <= ADC_CH5; ch ++)
			if(CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_DISABLE) == CN0411_FAILURE)
				return CN0411_FAILURE;

		if(CN0411_ADC_operation_mode(cn0411_dev, IDLE_MODE) == CN0411_FAILURE)
			return CN0411_FAILURE;

		break;
	case ADC_CONTINUOUS_CONV:
		for (int ch = ADC_CH0; ch <= ADC_CH5; ch ++)
			if(CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_ENABLE) == CN0411_FAILURE)
				return CN0411_FAILURE;

		if(CN0411_ADC_operation_mode(cn0411_dev, CONTINUOUS_CONV) == CN0411_FAILURE)
			return CN0411_FAILURE;

		break;
	default:
		return CN0411_FAILURE;
	}

	return CN0411_SUCCESS;
}

/**
 * CN0411 read ADC CHANNEL
 *
 * Based on the conversion mode used:
 * 	- continuous: since the function is called by the user and the sequence
 * 	  of channels conversions cannot be controlled individually, the function
 * 	  assures that the conversion data is read properly, gathering the data only
 * 	  after the transition is made on the desired ADC channel.
 * 	- single: read channel one channel individually
 *
 * @param cn0411_dev - The device structure.
 * @param ch - ADC channel
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_read_ch(struct cn0411_device *cn0411_dev, uint8_t ch)
{
	int32_t ret, timeout;
	uint32_t status_reg;

	switch (cn0411_dev->conv_type) {
	case ADC_CONTINUOUS_CONV:
		timeout = 4000000; /* 4000000 * 5uS = 20s */
		do {
			if(AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
					       &cn0411_dev->ad7124_dev.regs[AD7124_Status]) == CN0411_FAILURE)
				return CN0411_FAILURE;

			status_reg = cn0411_dev->ad7124_dev.regs[AD7124_Status].value & ADC_CH_RDY_MSK;
			timer_sleep_5uS(1u);
		} while((status_reg == ch) && (--timeout));

		if(timeout == 0)
			return CN0411_FAILURE;

		timeout = 4000000;
		do {
			if(AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
					       &cn0411_dev->ad7124_dev.regs[AD7124_Status]) == CN0411_FAILURE)
				return CN0411_FAILURE;

			status_reg = cn0411_dev->ad7124_dev.regs[AD7124_Status].value & ADC_CH_RDY_MSK;
			timer_sleep_5uS(1u);
		} while(status_reg != ch && (--timeout));

		if(timeout == 0)
			return CN0411_FAILURE;

		if(AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
				       &cn0411_dev->ad7124_dev.regs[AD7124_Data]) == CN0411_FAILURE)
			return CN0411_FAILURE;

		break;
	case ADC_SINGLE_CONV:
		ret = CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_ENABLE);
		if (ret == CN0411_FAILURE)
			return ret;

		ret = CN0411_ADC_operation_mode(cn0411_dev, SINGLE_CONV);
		if (ret == CN0411_FAILURE)
			return ret;

		if (AD7124_WaitForConvReady(&cn0411_dev->ad7124_dev, ADC_TIMEOUT) == TIMEOUT) {
			printf("TIMEOUT\n");
			return CN0411_FAILURE;
		}
		ret = AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
					  &cn0411_dev->ad7124_dev.regs[AD7124_Data]);
		if (ret == CN0411_FAILURE)
			return ret;

		ret = CN0411_ADC_set_ch (cn0411_dev, ch, ADC_CH_DISABLE);
		if (ret == CN0411_FAILURE)
			return ret;

		break;
	default:
		return CN0411_FAILURE;
	}

	return ret;
}

/**
 * CN0411 read temperature
 *
 * Reads channel 0 of the ADC
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_read_temp(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	uint32_t ch0_data;

	ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH0);
	if (ret == CN0411_FAILURE)
		return ret;

	ch0_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

	float resistance = (float)(ch0_data) * RTD_REF_RES / 0xFFFFFF;
	if(resistance < cn0411_dev->rtd_res)
		/* temp_rtd = a0 + a1 * r + a2 * r^2 + a3 * r^3 + a4 * r^4 +  a5 * r^5 */
		cn0411_dev->temp = -242.02 + 2.228 * resistance + (2.5859 * pow(10, -3))
				   * pow(resistance, 2) - (48260.0 * pow(10, -6))
				   * pow(resistance, 3) - (2.8183 * pow(10, -3))
				   * pow(resistance, 4) + (1.5243 * pow(10, -10))
				   * pow(resistance, 5);
	else
		/* temp_rtd = (-A + sqrt(A^2 - 4 * B (1 - r / R0)) / (2 * B) */
		cn0411_dev->temp = (-A + sqrt((pow(A, 2) - 4 * B * (1 - resistance
					       / cn0411_dev->rtd_res)))) / (2 * B);

	return ret;
}

/**
 * CN0411 read peak-to-peak voltage
 *
 * Reads channel 1 and 2 of the ADC
 *
 * @param cn0411_dev - The device structure.
 *
 * @return peak-to-peak voltage value
*/
int32_t CN0411_read_vpp(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	uint32_t ch1_data = 0, ch2_data = 0;

	ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH1);
	if (ret == CN0411_FAILURE)
		return ret;

	ch1_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;
	ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH2);
	if (ret == CN0411_FAILURE)
		return ret;

	ch2_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

	cn0411_dev->vin_p = (ch1_data * VREF / 0xFFFFFF) / INSTR_AMP_GAIN;
	cn0411_dev->vin_n = (ch2_data * VREF / 0xFFFFFF) / INSTR_AMP_GAIN;
	cn0411_dev->vpp = cn0411_dev->vin_p + cn0411_dev->vin_n;

	return ret;
}

/**
 * CN0411 read DAC value
 *
 * Reads channel 3 of the ADC
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_read_vdac(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	uint32_t ch3_data = 0;

	ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH3);
	if (ret == CN0411_FAILURE)
		return ret;

	ch3_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

	cn0411_dev->read_dac = ch3_data * VREF / 0xFFFFFF;

	return ret;
}

/**
 * CN0411 read voltage value hooked to R20S
 *
 * Reads channel 4 of the ADC
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_read_R20S(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	uint32_t ch4_data = 0;

	ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH4);
	if (ret == CN0411_FAILURE)
		return ret;

	ch4_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

	cn0411_dev->read_v_r20s = ch4_data * VREF / 0xFFFFFF;

	return ret;
}

/**
 * CN0411 read voltage value hooked to R200S
 *
 * Reads channel 5 of the ADC
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_read_R200S(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	uint32_t ch5_data = 0;

	ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH5);
	if (ret == CN0411_FAILURE)
		return ret;

	ch5_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

	cn0411_dev->read_v_r200s = ch5_data * VREF / 0xFFFFFF;

	return ret;
}

/**
 * CN0411 compute input resistance
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_compute_rdres(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	float ipp;

	ret = CN0411_read_vpp(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ipp = (2*cn0411_dev->v_dac - cn0411_dev->vpp)
	      / cn0411_dev->r_gain[cn0411_dev->ch_gain];
	cn0411_dev->rdres = cn0411_dev->vpp/ipp;

	return ret;
}

/**
 * CN0411 compute electric conductivity
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_compute_cond(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	float ipp, g;

	ret = CN0411_premeasurement(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_read_vpp(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ipp = (2*cn0411_dev->v_exc - cn0411_dev->vpp)
	      / cn0411_dev->r_gain[cn0411_dev->ch_gain];
	cn0411_dev->rdres = cn0411_dev->vpp/ipp - (cn0411_dev->offset_res);
	g = 1/cn0411_dev->rdres;
	cn0411_dev->cond = cn0411_dev->cell_const * g;

	return ret;
}

/**
 * CN0411 compute offset resistance
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_compute_off_res(struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	float ipp;

	ret = CN0411_premeasurement(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_read_vpp(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ipp = (2*cn0411_dev->v_exc - cn0411_dev->vpp)
	      / cn0411_dev->r_gain[cn0411_dev->ch_gain];
	cn0411_dev->offset_res = cn0411_dev->vpp/ipp - PREC_REF_RES;

	return ret;
}

/**
 * CN0411 temperature compensation of conductivity
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_compensate_cond(struct cn0411_device *cn0411_dev)
{
	int32_t ret;

	ret = CN0411_read_temp(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_compute_cond(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	cn0411_dev->comp_cond = cn0411_dev->cond
				/ (1 + cn0411_dev->solution.temp_coeff
				   * (cn0411_dev->temp - TCAL));

	return ret;
}

/**
 * CN0411 compute TDS
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_compute_tds(struct cn0411_device *cn0411_dev)
{
	if(CN0411_compensate_cond(cn0411_dev) == CN0411_FAILURE)
		return CN0411_FAILURE;

	cn0411_dev->tds = cn0411_dev->solution.tds_factor * cn0411_dev->comp_cond;

	return CN0411_SUCCESS;
}

/**
 * CN0411 ADC channels internal software calibration
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_int_calibrate(struct cn0411_device *cn0411_dev)
{
	int32_t ret;

	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH0, ADC_CH_ENABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_operation_mode(cn0411_dev, CAL_INT_ZERO_MODE);
	if (ret == CN0411_FAILURE)
		return ret;

	timer_sleep(1000u);
	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH0, ADC_CH_DISABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH1, ADC_CH_ENABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_operation_mode(cn0411_dev, CAL_INT_ZERO_MODE);
	if (ret == CN0411_FAILURE)
		return ret;

	timer_sleep(1000u);
	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH1, ADC_CH_DISABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH2, ADC_CH_ENABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_operation_mode(cn0411_dev, CAL_INT_ZERO_MODE);
	if (ret == CN0411_FAILURE)
		return ret;

	timer_sleep(1000u);
	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH2, ADC_CH_DISABLE);

	return ret;
}

/**
 * CN0411 ADC channels system calibration
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_ADC_sys_calibrate(struct cn0411_device *cn0411_dev)
{
	int32_t ret;

	ret = CN0411_ADC_operation_mode(cn0411_dev, IDLE_MODE);
	if (ret == CN0411_FAILURE)
		return ret;

	for(uint8_t ch = ADC_CH0; ch <= ADC_CH2; ch ++) {
		ret = CN0411_ADC_set_ch (cn0411_dev, ch, ADC_CH_DISABLE);
		if (ret == CN0411_FAILURE)
			return ret;
	}
	pwm_status = PWM_SYSCALIB_AIN7;
	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH1, ADC_CH_ENABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	timer_sleep(1000u);
	ret = CN0411_ADC_operation_mode(cn0411_dev, CAL_SYS_ZERO_MODE);
	if (ret == CN0411_FAILURE)
		return ret;

	timer_sleep(1000u);
	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH1, ADC_CH_DISABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	pwm_status = PWM_SYSCALIB_AIN8;
	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH2, ADC_CH_ENABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	timer_sleep(1000u);
	ret = CN0411_ADC_operation_mode(cn0411_dev, CAL_SYS_ZERO_MODE);
	if (ret == CN0411_FAILURE)
		return ret;

	timer_sleep(1000u);
	ret = CN0411_ADC_set_ch (cn0411_dev, ADC_CH2, ADC_CH_DISABLE);
	if (ret == CN0411_FAILURE)
		return ret;

	pwm_status = PWM_CONVERSION;
	ret = CN0411_ADC_conv_init (cn0411_dev, cn0411_dev->conv_type);

	return ret;
}

/**
 * CN0411 Premeasurement Process
 *
 * Autoset gain resistor channel based on the read DAC value
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_premeasurement(struct cn0411_device *cn0411_dev)
{
	int32_t ret;

	cn0411_dev->ch_gain = CH_GAIN_RES_20M;
	ret = CN0411_read_vdac(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	cn0411_dev->v_exc = cn0411_dev->read_dac;
	ret = CN0411_DAC_set_value(cn0411_dev, cn0411_dev->v_exc);
	if (ret == CN0411_FAILURE)
		return ret;

	while (cn0411_dev->ch_gain >= 1) {
		ret = CN0411_ADC_set_io1 (cn0411_dev, cn0411_dev->ch_gain);
		if (ret == CN0411_FAILURE)
			return ret;

		ret = CN0411_read_vpp(cn0411_dev);
		if (ret == CN0411_FAILURE)
			return ret;

		if((cn0411_dev->vpp > 0.3 * 2 * cn0411_dev->v_exc)
		    || (cn0411_dev->ch_gain == 1)) {
			cn0411_dev->v_exc = 0.4 * cn0411_dev->v_exc / cn0411_dev->vpp;
			ret = CN0411_DAC_set_value(cn0411_dev, cn0411_dev->v_exc);
			if (ret == CN0411_FAILURE)
				return ret;

			break;
		} else {
			cn0411_dev->ch_gain--;
		}
	}

	return ret;
}

/**
 * CN0411 Set Gain Resistance
 *
 * @param cn0411_dev - The device structure.
 * @param gain_res - gain resistor channel value
 * @return 0 in case of success, negative error code otherwise.
*/

int32_t CN0411_set_gain_res(struct cn0411_device *cn0411_dev, int8_t ch_gain)
{
	cn0411_dev->ch_gain = ch_gain;

	return CN0411_ADC_set_io1 (cn0411_dev, cn0411_dev->ch_gain);
}
/**
 * Find available commands
 *
 * @param cmd - command to search
 * @return cmd_func - return the specific function for available command or
 * NULL for invalid command
*/
cmd_func CN0411_find_command(char *cmd)
{
	cmd_func func = NULL;
	int i = 0;

	while (cmd_fun[i] != NULL) {
		if (strncmp(cmd, cmd_commands[i], 6) == 0) {
			func = cmd_fun[i];
			break;
		}

		i++;
	}

	return func;
}

/**
 * Command line process function
 *
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_process(struct cn0411_device *cn0411_dev)
{
	cmd_func func;

	/* Check if <ENTER> key was pressed */
	if (uart_cmd == UART_TRUE) {

		/* Find needed function based on typed command */
		func = CN0411_find_command((char *)uart_rx_buffer);

		/* Check if there is a valid command */
		if (func) {
			printf("\n");
			/* Call the desired function */
			(*func)(&uart_rx_buffer[2], cn0411_dev);

			/* Check if there is no match for typed command */
		} else if (strlen((char *)uart_rx_buffer) != 0) {
			printf("\n");
			/* Display a message for unknown command */
			printf("Unknown command!");
			printf("\n");
		}

		/* Prepare for next <ENTER> */
		uart_cmd = UART_FALSE;
		CN0411_cmd_prompt();
	}
}

/**
 * Command line prompt
 *
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_cmd_prompt(void)
{
	int32_t res;
	static uint8_t count = 0;

	res = UART_WriteChar(_CR, UART_WRITE_NO_INT);
	if (res == UART_SUCCESS) {
		res = UART_WriteChar(_LF, UART_WRITE_NO_INT);
	}
	/* Check first <ENTER> is pressed after reset */
	if(count == 0) {
		printf("\tWelcome to CN0411 application!\n");
		printf("Type <help> to see available commands...\n");
		printf("\n");
		count++;
	}
	if (res == UART_SUCCESS) {
		UART_WriteChar(':', UART_WRITE_NO_INT);
	}
	uart_rcnt = 0;

	return res;
}

/**
 * Finds the next command line argument
 *
 * @param args - pointer to the arguments on the command line.
 * @return pointer to the next argument.
*/
uint8_t *CN0411_find_argv(uint8_t *args)
{
	uint8_t *p = args;
	int fl = 0;

	while (*p != 0) {
		if ((*p == _SPC)) {
			fl = 1;
		} else {
			if (fl) {
				break;
			}
		}
		p++;
	}

	return p;
}

/**
 * Separates a command line argument
 *
 * @param dst - pointer to a buffer where the argument will be copied
 * @param args - pointer to the current position of the command line .
 * @return none
*/
void CN0411_get_argv(char *dst, uint8_t *args)
{
	uint8_t *s = args;
	char *d = dst;

	while (*s) {
		if (*s == _SPC) {
			break;
		}
		*d++ = *s++;
	}
	*d = '\0';
}


/**
 * Display info for <help> command
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_help(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	printf("\n");
	printf("Available commands:\n");
	printf(" help                          - Display available commands\n");
	printf(" syscal                        - Perform ADC system zero-scale "
	       "calibration \n");
	printf("                                 Before calibration, short "
	       "terminals 5 & 6 \n");
	printf("                                 in jumper P5. \n");
	printf(" refres                        - Perform Referencing to a "
	       "Precision Resistance \n");
	printf("                                 Before referencing, short "
	       "terminals 3 & 4 \n");
	printf("                                 in jumper P5. \n");
	printf(" convmod (sing/cont)           - set single/continuous conversion "
	       "mode for ADC \n");
	printf(" autoset                       - Autoset Gain Resistance.\n");
	printf(" setdac <val>                  - Set DAC value (Volts) \n");
	printf("                                 <val> = values from 0 to 2.5 \n");
	printf(" gainres <val>                 - Set Gain Resistor value (Ω) \n");
	printf("                                 <val> = 20/200/2K/20K/200K/2M/20M"
	       " \n");
	printf(" rtdval <val>                  - Set RTD value (Ω) \n");
	printf("                                 <val> = values 100, 1000 \n");
	printf(" pwmfreq <val>                 - Set PWM frequency value (Hz) \n");
	printf("                                 <val> = values 94, 2400 \n");
	printf(" cellconst (low/normal/high)   - set cell constant for "
	       "conductivity types\n");
	printf(" solution (kcl/nacl)           - set parameters for specific "
	       "solution \n");
	printf(" temp                          - Display temperature value\n");
	printf(" vinput (pos/neg)              - Display Positive/Negative input "
	       "voltage \n");
	printf(" readdac                       - Read DAC value (Volts) \n");
	printf(" rdr20s                        - Read Voltage on R20S (Volts) \n");
	printf(" rdr200s                       - Read Voltage on R200S (Volts) \n");
	printf(" rdres                         - Read Input Resistance (Volts) \n");
	printf(" cond                          - Display conductivity value \n");
	printf(" tds                           - Display TDS value \n");
}

/**
 * Perform System Calibration via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_sys_calib(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	printf("ADC System Calibration in progress...\n");

	if(CN0411_ADC_sys_calibrate(cn0411_dev) == CN0411_FAILURE)
		printf("ADC System Calibration failed!\n");
	else
		printf("ADC System Calibration completed!\n");
}

/**
 * Referencing to a Precision Resistance via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_off_res(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	printf("Referencing to a Precision Resistance in progress...\n");
	if(CN0411_compute_off_res(cn0411_dev) == CN0411_FAILURE)
		printf("Referencing to a Precision Resistance failed!\n");
	else
		printf("Referencing to a Precision Resistance completed!\n");
}

/**
 * Set Conversion Mode via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_conv_mode(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	uint8_t *p = args;
	char arg[5];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0') {
		/* Save conversion mode parameter */
		CN0411_get_argv(arg, p);
	}
	if(!strcmp(arg, "sing")) {
		cn0411_dev->conv_type = ADC_SINGLE_CONV;
		ret = CN0411_ADC_conv_init(cn0411_dev, cn0411_dev->conv_type);
		printf("ADC set to single Conversion Mode.\n");
	} else if (!strcmp(arg, "cont")) {
		cn0411_dev->conv_type = ADC_CONTINUOUS_CONV;
		ret = CN0411_ADC_conv_init(cn0411_dev, cn0411_dev->conv_type);
		printf("ADC set to continuous Conversion Mode.\n");
	} else {
		ret = CN0411_FAILURE;
		printf("Incorrect input value!\n");
	}
	if (ret != CN0411_SUCCESS) {
		printf("Conversion Mode initialization failed!\n");
	}
}

/**
 * Autoset Gain Resistance via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_autoset(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	printf("Autoset Gain Resistance in progress...\n");

	if(CN0411_premeasurement(cn0411_dev) == CN0411_FAILURE)
		printf("Autoset Gain Resistance failed!\n");
	else
		printf("Autoset Gain Resistance completed!\n");
}

/**
 * Set DAC value via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_set_dac(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	int32_t ret;
	uint8_t *p = args;
	float input_val;
	char arg[5];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0') {
		/* Save DAC value */
		CN0411_get_argv(arg, p);
	}
	input_val = atof(arg);
	if(input_val < 0 || input_val > 2.5) {
		ret = CN0411_FAILURE;
		printf("Input out of range!.\n");
	} else {
		cn0411_dev->v_dac = input_val;
		ret = CN0411_DAC_set_value(cn0411_dev, cn0411_dev->v_dac);
		printf("DAC value set to %s V.\n", arg);
	}
	if (ret != CN0411_SUCCESS) {
		printf("Set DAC value failed!.\n");
	}
}

/**
 * Set gain resistance via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_gain_res(uint8_t *args, struct cn0411_device *cn0411_dev)
{

	int32_t ret = 0;
	uint8_t *p = args;
	char arg[5];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0') {
		/* Save gain resistor value */
		CN0411_get_argv(arg, p);
	}

	if(!strcmp(arg, "20M")) {
		ret = CN0411_set_gain_res(cn0411_dev, CH_GAIN_RES_20M);
		printf("Gain Resistor set to 20M.\n");
	} else if(!strcmp(arg, "2M")) {
		ret = CN0411_set_gain_res(cn0411_dev, CH_GAIN_RES_2M);
		printf("Gain Resistor set to 2M.\n");
	} else if(!strcmp(arg, "200K")) {
		ret = CN0411_set_gain_res(cn0411_dev, CH_GAIN_RES_200K);
		printf("Gain Resistor set to 200K.\n");
	} else if(!strcmp(arg, "20K")) {
		ret = CN0411_set_gain_res(cn0411_dev, CH_GAIN_RES_20K);
		printf("Gain Resistor set to 20K.\n");
	} else if(!strcmp(arg, "2K")) {
		ret = CN0411_set_gain_res(cn0411_dev, CH_GAIN_RES_2K);
		printf("Gain Resistor set to 2K.\n");
	} else if(!strcmp(arg, "200")) {
		ret = CN0411_set_gain_res(cn0411_dev, CH_GAIN_RES_200);
		printf("Gain Resistor set to 200.\n");
	} else if(!strcmp(arg, "20")) {
		ret = CN0411_set_gain_res(cn0411_dev, CH_GAIN_RES_20);
		printf("Gain Resistor set to 20.\n");
	} else {
		printf("Invalid argument!\n");
	}
	if (ret == CN0411_FAILURE) {
		printf("Set gain resistor value failed!\n");
	}
}

/**
 * Set RTD resistance value via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/

void CN0411_cmd_rtd_val(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	uint8_t *p = args;
	int input_val;
	char arg[6];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0') {
		/* Save RTD value parameter */
		CN0411_get_argv(arg, p);
	}
	input_val = atoi(arg);
	if(input_val == 100) {
		cn0411_dev->rtd_res = RTD_RES_100;
		printf("RTD value set to 100Ω.\n");
	} else if(input_val == 1000) {
		cn0411_dev->rtd_res = RTD_RES_1K;
		printf("RTD value set to 1kΩ.\n");
	} else {
		printf("Incorrect value!\n");
	}
}

/**
 * Set PWM frequency via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/

void CN0411_cmd_pwm_freq(uint8_t *args, struct cn0411_device * cn0411_dev)
{
	uint8_t *p = args;
	int input_val;
	char arg[6];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0') {
		/* Save PWM frequency parameter */
		CN0411_get_argv(arg, p);
	}
	input_val = atoi(arg);
	if(input_val == PWM_FREQ_94 || input_val == PWM_FREQ_2400) {
		CN0411_pwm_freq(input_val);
		if(input_val == PWM_FREQ_94)
			printf("PWM frequency set to 94Hz.\n");
		else
			printf("PWM frequency set to 2.4KHz.\n");
	} else {
		printf("Incorrect value!\n");
	}
}

/**
 * Set Cell Constant type via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/

void CN0411_cmd_cell_const(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	uint8_t *p = args;
	char arg[7];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0') {
		/* Save cell constant parameter */
		CN0411_get_argv(arg, p);
	}
	if(!strcmp(arg, "low")) {
		cn0411_dev->cell_const = CELL_CONST_LOW;
		printf("Cell Constant set to low.\n");
	} else if (!strcmp(arg, "normal")) {
		cn0411_dev->cell_const = CELL_CONST_NORMAL;
		printf("Cell Constant set to normal.\n");
	} else if (!strcmp(arg, "high")) {
		cn0411_dev->cell_const = CELL_CONST_HIGH;
		printf("Cell Constant set to high.\n");
	} else {
		printf("Incorrect input value!\n");
	}
}

/**
 * Set Solution parameters via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_solution(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	uint8_t *p = args;
	char arg[5];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0') {
		/* Save solution parameter */
		CN0411_get_argv(arg, p);
	}
	if(!strcmp(arg, "kcl")) {
		cn0411_dev->solution.temp_coeff = TEMP_COEFF_KCL;
		cn0411_dev->solution.tds_factor = TDS_KCL;
		printf("Solution set to KCl.\n");
	} else if (!strcmp(arg, "nacl")) {
		cn0411_dev->solution.temp_coeff = TEMP_COEFF_NACL;
		cn0411_dev->solution.tds_factor = TDS_NACL;
		printf("Solution set to NaCl.\n");
	} else {
		printf("Incorrect input value!\n");
	}
}

/**
 * Display Temperature value via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_temp(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	if(CN0411_read_temp(cn0411_dev) == CN0411_FAILURE) {
		printf("Get Temperature value failed!\n");
	} else {
		printf("Temperature = %.2f[°C]\n", cn0411_dev->temp);
	}
}

/**
 * Display Positive/Negative Input Voltage via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_vinput(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	uint8_t *p = args;
	char arg[5];

	/* Check if this function gets an argument */
	while (*(p = CN0411_find_argv(p)) != '\0')
		/* Save input value */
		CN0411_get_argv(arg, p);

	if(CN0411_read_vpp(cn0411_dev) == CN0411_FAILURE)
		printf("Get Input Voltage value failed!\n");
	else {
		if(!strcmp(arg, "pos"))
			printf("Positive Input Voltage = %.5f V\n", cn0411_dev->vin_p);
		else if(!strcmp(arg, "neg"))
			printf("Negative Input Voltage = %.5f V\n", cn0411_dev->vin_n);
		else
			printf("Incorrect input value!\n");
	}
}

/**
 * Display DAC value via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_read_dac(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	if(CN0411_read_vdac(cn0411_dev) == CN0411_FAILURE)
		printf("Read DAC Voltage value failed!\n");
	else
		printf("DAC value = %.3f V\n", cn0411_dev->read_dac);
}

/**
 * Display voltage on R20s
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_read_20s(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	int32_t ret;

	ret = CN0411_read_R20S(cn0411_dev);
	ret |= CN0411_read_vdac(cn0411_dev);
	if(ret == CN0411_SUCCESS)
		printf("Voltage on R20S value = %.3f V\n",
		       (cn0411_dev->read_dac - cn0411_dev->read_v_r20s));
	else
		printf("Read Voltage on R20S Value failed!\n");
}

/**
 * Display voltage on R200s
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_read_200s(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	int32_t ret;

	ret = CN0411_read_R200S(cn0411_dev);
	ret |= CN0411_read_vdac(cn0411_dev);

	if(ret == CN0411_SUCCESS)
		printf("Voltage on R200S value = %.3f V\n",
		       (cn0411_dev->read_dac - cn0411_dev->read_v_r200s));
	else
		printf("Read Voltage on R200S Value failed!\n");
}

/**
 * Display input resistance
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_rdres(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	if(CN0411_compute_rdres(cn0411_dev) == CN0411_FAILURE)
		printf("Input resistance value failed!\n");
	else
		printf("Input resistance value = %.3f Ω\n", cn0411_dev->rdres);
}

/**
 * Display Conductivity value via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_cond(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	printf("Conductivity measurement in progress...\n");

	if(CN0411_compensate_cond(cn0411_dev) == CN0411_FAILURE)
		printf("Get Conductivity value failed!\n");
	else
		printf("Conductivity = %.10f\n", cn0411_dev->cond);
}

/**
 * Display TDS value via UART
 *
 * @param args - pointer to the arguments on the command line.
 * @param cn0411_dev - The device structure.
 * @return none
*/
void CN0411_cmd_tds(uint8_t *args, struct cn0411_device *cn0411_dev)
{
	printf("TDS measurement in progress...\n");

	if(CN0411_compute_tds(cn0411_dev) == CN0411_FAILURE)
		printf("Get TDS value failed!\n");
	else
		printf("TDS = %.10f\n", cn0411_dev->tds);
}

/**
 * Internal interrupt handler for UART

 * @return none
*/
void CN0411_interrupt(void)
{
	unsigned short status;
	char c;

	status = UrtIntSta(pADI_UART);
	if (status & COMIIR_NINT)
		return;   /* Check if UART is busy */

	switch (status & COMIIR_STA_MSK) {
	case COMIIR_STA_RXBUFFULL:
		UART_ReadChar(&c);
		switch(c) {
		case _BS:
			if (uart_rcnt) {
				uart_rcnt--;
				uart_rx_buffer[uart_rcnt] = 0;
				UART_WriteChar(c, UART_WRITE_IN_INT);
				UART_WriteChar(' ', UART_WRITE_IN_INT);
				UART_WriteChar(c, UART_WRITE_IN_INT);
			}
			break;
		case _CR: /* Check if read character is ENTER */
			uart_cmd = UART_TRUE;                    /* Set flag */
			break;
		default:
			uart_rx_buffer[uart_rcnt++] = c;

			if (uart_rcnt == UART_RX_BUFFER_SIZE) {
				uart_rcnt--;
				UART_WriteChar(_BS, UART_WRITE_IN_INT);
			}
			UART_WriteChar(c, UART_WRITE_IN_INT);
		}
		uart_rx_buffer[uart_rcnt] = '\0';
		break;
	case COMIIR_STA_TXBUFEMPTY:
		if (uart_tcnt) {
			uart_tbusy = UART_TRUE;
			uart_tcnt--;
			UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);
			if (uart_tpos == UART_TX_BUFFER_SIZE) {
				uart_tpos = 0;
			}
		} else {
			uart_tbusy = UART_FALSE;
		}
		break;
	default:
		;
	}
}

/**
 * Set PWM frequency
 *
 * @param freq - frequency value to be set.
 * @return none
*/
void CN0411_pwm_freq(uint16_t freq)
{
	switch (freq) {
	case PWM_FREQ_94:
		pwm_freq = pwm_100_freq;
		pwm_tick_count = 0;
		pwm_index = 0;
		break;
	case PWM_FREQ_2400:
		pwm_freq = pwm_2400_freq;
		pwm_tick_count = 0;
		pwm_index = 0;
		break;
	default:
		break;
	}
}

/**
 * Generate PWM

 * @return none
*/
void CN0411_pwm_gen(void)
{
	switch(pwm_status) {
	case PWM_SYSCALIB_AIN7:
		DioClr(pADI_GP1, BIT4);
		DioSet(pADI_GP1, BIT2|BIT3);
		break;
	case PWM_SYSCALIB_AIN8:
		DioClr(pADI_GP1, BIT2|BIT3);
		DioSet(pADI_GP1, BIT4);
		break;
	case PWM_CONVERSION:
		if(pwm_tick_count < pwm_freq[pwm_index])
			pwm_tick_count++;
		else {
			pSetClr[pwm_setclr[pwm_index]](pADI_GP1, pwm_bit[pwm_index]);
			if(pwm_index >= ARRAY_SIZE(pwm_100_freq)-1) {
				pwm_index = 0;
				pwm_tick_count = 0;
			} else {
				pwm_index++;
				pwm_tick_count++;
			}
		}
		break;
	default:
		break;
	}
}

/**
 * CN0411 Initialization
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
*/
int32_t CN0411_init(struct cn0411_device *cn0411_dev,
		    struct cn0411_init_params cn0411_init_params)
{
	int32_t ret;

	/* Initialize Device Structure */
	cn0411_dev->ch_gain = cn0411_init_params.init_ch_gain;
	cn0411_dev->conv_type = cn0411_init_params.init_conv_type;
	cn0411_dev->rtd_res = cn0411_init_params.init_rtd_res;
	cn0411_dev->r_gain[CH_GAIN_RES_20] =
		cn0411_init_params.init_r_gain[CH_GAIN_RES_20];
	cn0411_dev->r_gain[CH_GAIN_RES_200] =
		cn0411_init_params.init_r_gain[CH_GAIN_RES_200];
	cn0411_dev->r_gain[CH_GAIN_RES_2K] =
		cn0411_init_params.init_r_gain[CH_GAIN_RES_2K];
	cn0411_dev->r_gain[CH_GAIN_RES_20K] =
		cn0411_init_params.init_r_gain[CH_GAIN_RES_20K];
	cn0411_dev->r_gain[CH_GAIN_RES_200K] =
		cn0411_init_params.init_r_gain[CH_GAIN_RES_200K];
	cn0411_dev->r_gain[CH_GAIN_RES_2M] =
		cn0411_init_params.init_r_gain[CH_GAIN_RES_2M];
	cn0411_dev->r_gain[CH_GAIN_RES_20M] =
		cn0411_init_params.init_r_gain[CH_GAIN_RES_20M];
	cn0411_dev->offset_res = cn0411_init_params.init_offset_res;
	cn0411_dev->v_dac = cn0411_init_params.init_v_dac;
	cn0411_dev->read_dac = cn0411_init_params.init_read_dac;
	cn0411_dev->read_v_r20s = cn0411_init_params.init_read_v_r20s;
	cn0411_dev->read_v_r200s = cn0411_init_params.init_read_v_r200s;
	cn0411_dev->rdres = cn0411_init_params.init_rdres;
	cn0411_dev->v_exc = cn0411_init_params.init_v_exc;
	cn0411_dev->cell_const = cn0411_init_params.init_cell_const;
	cn0411_dev->temp = cn0411_init_params.init_temp;
	cn0411_dev->vin_p = cn0411_init_params.init_vin_p;
	cn0411_dev->vin_n = cn0411_init_params.init_vin_n;
	cn0411_dev->cond = cn0411_init_params.init_cond;
	cn0411_dev->comp_cond = cn0411_init_params.init_comp_cond;
	cn0411_dev->tds = cn0411_init_params.init_tds;
	cn0411_dev->solution.tds_factor =
		cn0411_init_params.init_solution.init_tds_factor;
	cn0411_dev->solution.temp_coeff =
		cn0411_init_params.init_solution.init_temp_coeff;

	/* Initialize PWM */
	pwm_2400_freq[0] = PWM2_2400_HIGH;
	pwm_2400_freq[1] = PWM2_2400_LOW;
	pwm_2400_freq[2] = PWM0_2400_HIGH;
	pwm_2400_freq[3] = PWM1_2400_HIGH;
	pwm_2400_freq[4] = PWM1_2400_LOW;
	pwm_2400_freq[5] = PWM0_2400_LOW;

	pwm_100_freq[0] = PWM2_100_HIGH;
	pwm_100_freq[1] = PWM2_100_LOW;
	pwm_100_freq[2] = PWM0_100_HIGH;
	pwm_100_freq[3] = PWM1_100_HIGH;
	pwm_100_freq[4] = PWM1_100_LOW;
	pwm_100_freq[5] = PWM0_100_LOW;

	pSetClr = (PWM_SETCLR *)malloc(sizeof(PWM_SETCLR *) * 2);
	pSetClr[0] = (PWM_SETCLR)&DioClr;
	pSetClr[1] = (PWM_SETCLR)&DioSet;

	pwm_setclr[0] = PWM_SET;
	pwm_setclr[1] = PWM_CLR;
	pwm_setclr[2] = PWM_SET;
	pwm_setclr[3] = PWM_SET;
	pwm_setclr[4] = PWM_CLR;
	pwm_setclr[5] = PWM_CLR;

	pwm_bit[0] = BIT4;
	pwm_bit[1] = BIT4;
	pwm_bit[2] = BIT2;
	pwm_bit[3] = BIT3;
	pwm_bit[4] = BIT3;
	pwm_bit[5] = BIT2;

	CN0411_pwm_freq(PWM_FREQ_94);
	pwm_status = PWM_CONVERSION;

	/* Initial Setup ADC */
	ret = AD7124_Setup(&cn0411_dev->ad7124_dev, CS_AD7124, ad7124_regs);
	if (ret == CN0411_FAILURE)
		return ret;

	/* Initial Setup DAC */
	ret = AD5683_setup(&cn0411_dev->ad5683_dev, CS_AD5683);
	if (ret == CN0411_FAILURE)
		return ret;

	/* Setup ADC */
	ret = CN0411_ADC_setup (cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_set_io1 (cn0411_dev, cn0411_dev->ch_gain);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_set_io2 (cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_int_calibrate(cn0411_dev);
	if (ret == CN0411_FAILURE)
		return ret;

	ret = CN0411_ADC_conv_init(cn0411_dev, cn0411_dev->conv_type);
	if (ret == CN0411_FAILURE)
		return ret;

	/* Set DAC output value */
	ret = CN0411_DAC_set_value(cn0411_dev, cn0411_dev->v_dac);
	if (ret == CN0411_FAILURE)
		return ret;

	printf("CN0411 Successfully Initialized!\n");
	printf("CN0411 Initial Setup:\n");
	printf("	- ADC set to Single Conversion Mode\n");
	printf("	- DAC output voltage set to 0.4V\n");
	printf("	- PWM frequency set to 94Hz\n");
	printf("	- RTD resistance set to 100Ω\n");
	printf("	- Cell Constant set to normal\n");
	printf("	- Solution set to NaCl\n");

	return ret;
}
