/***************************************************************************//**
 *   @file   CN0411.h
 *   @brief  CN0411 header file
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
#ifndef _CN0411_H_
#define _CN0411_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "AD5683.h"
#include "AD7124.h"
#include <adi_processor.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define CN0411_SUCCESS 0
#define CN0411_FAILURE -1
#define CS_AD7124 0
#define CS_AD5683 1
#define INSTR_AMP_GAIN 10
#define ADC_TIMEOUT 10000
#define ADC_CH_DISABLE 0
#define ADC_CH_ENABLE 1
#define ADC_SETUP0 0
#define ADC_SETUP1 1
#define ADC_CH0 0
#define ADC_CH1 1
#define ADC_CH2 2
#define ADC_CH3 3
#define ADC_CH4 4
#define ADC_CH5 5
#define ADC_CONTINUOUS_CONV 1
#define ADC_SINGLE_CONV 2
#define ADC_CH_RDY_MSK 0x8F
#define PWM_SYSCALIB_AIN7 1
#define PWM_SYSCALIB_AIN8 2
#define PWM_CONVERSION 3
#define PWM_FREQ_94 94
#define PWM_FREQ_2400 2400
#define PWM_2400_STEP 6
#define PWM2_2400_HIGH (1 * PWM_2400_STEP)
#define PWM2_2400_LOW (2 * PWM_2400_STEP)
#define PWM0_2400_HIGH (3 * PWM_2400_STEP)
#define PWM1_2400_HIGH (4 * PWM_2400_STEP)
#define PWM1_2400_LOW (5 * PWM_2400_STEP)
#define PWM0_2400_LOW (6 * PWM_2400_STEP)
#define PWM_100_STEP 161
#define PWM2_100_HIGH (1 * PWM_100_STEP)
#define PWM2_100_LOW (2 * PWM_100_STEP)
#define PWM0_100_HIGH (3 * PWM_100_STEP)
#define PWM1_100_HIGH (4 * PWM_100_STEP)
#define PWM1_100_LOW (5 * PWM_100_STEP)
#define PWM0_100_LOW (6 * PWM_100_STEP)
#define PWM_CLR 0
#define PWM_SET 1
#define ADC_SET_CH(x) (x << 15)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define DAC_FS_VAL 0xFFFF
#define CH_GAIN_RES_20 0
#define CH_GAIN_RES_200 1
#define CH_GAIN_RES_2K 2
#define CH_GAIN_RES_20K 3
#define CH_GAIN_RES_200K 4
#define CH_GAIN_RES_2M 5
#define CH_GAIN_RES_20M 6
#define GAIN_RES_20 20
#define GAIN_RES_200 200
#define GAIN_RES_2K 2000
#define GAIN_RES_20K 20000
#define GAIN_RES_200K 200000
#define GAIN_RES_2M 2000000
#define GAIN_RES_20M 20000000
#define DAC_OUT_DEFAULT_VAL 0.4
#define DAC_IN_DEFAULT_VAL 0
#define TEMP_DEFAULT_VAL 0
#define VPP_DEFAULT_VAL 0
#define VINP_DEFAULT_VAL 0
#define VINN_DEFAULT_VAL 0
#define VR20S_DEFAULT_VAL 0
#define VR200S_DEFAULT_VAL 0
#define COND_DEFAULT_VAL 0
#define COMP_COND_DEFAULT_VAL 0
#define TDS_DEFAULT_VAL 0
#define EXC_DEFAULT_VAL 0
#define RES_GAIN_DEFAULT_CH 6
#define VREFIN (0.250 * 4.02)
#define OFFSET_RES_INIT 0
#define RDRES_DEFAULT_VAL 0
#define PREC_REF_RES 1500
#define RTD_REF_RES 4020
#define RTD_RES_100 100
#define RTD_RES_1K 1000
#define TCAL 25
#define CELL_CONST_LOW 0.1
#define CELL_CONST_NORMAL 1
#define CELL_CONST_HIGH 10
#define TDS_KCL 0.5
#define TDS_NACL 0.47
#define TEMP_COEFF_KCL 1.88
#define TEMP_COEFF_NACL 2.14
#define VREF 2.5
#define A (3.9083*pow(10,-3))
#define B (-5.775*pow(10,-7))

/******************************************************************************/
/************************** Variable Declaration ******************************/
/******************************************************************************/

typedef uint32_t (*PWM_SETCLR) (ADI_GPIO_TypeDef *pPort, uint32_t iVal);
PWM_SETCLR *pSetClr;

uint8_t pwm_status;
uint8_t pwm_index;
uint32_t pwm_tick_count;

uint16_t pwm_2400_freq[6];
uint16_t pwm_100_freq[6];
uint16_t pwm_bit[6];
uint16_t pwm_setclr[6];

uint16_t *pwm_freq;

struct solution {
	float temp_coeff;
	float tds_factor;
};

struct init_solution {
	float init_temp_coeff;
	float init_tds_factor;
};

struct cn0411_device {
	uint8_t ch_gain;
	uint8_t conv_type;
	uint16_t rtd_res;
	uint32_t r_gain[7];
	float offset_res;
	float v_dac;
	float read_dac;
	float read_v_r20s;
	float read_v_r200s;
	float rdres;
	float v_exc;
	float cell_const;
	float temp;
	float vpp;
	float vin_p;
	float vin_n;
	float cond;
	float comp_cond;
	float tds;
	struct solution solution;
	struct ad7124_device ad7124_dev;
	struct ad5683_device ad5683_dev;
};

struct cn0411_init_params {
	uint8_t init_ch_gain;
	uint8_t init_conv_type;
	uint16_t init_rtd_res;
	uint32_t init_r_gain[7];
	float init_offset_res;
	float init_v_dac;
	float init_read_dac;
	float init_read_v_r20s;
	float init_read_v_r200s;
	float init_rdres;
	float init_v_exc;
	float init_cell_const;
	float init_temp;
	float init_vpp;
	float init_vin_p;
	float init_vin_n;
	float init_cond;
	float init_comp_cond;
	float init_tds;
	struct init_solution init_solution;
};

typedef  void (*cmd_func)(uint8_t *, struct cn0411_device *);

/******************************************************************************/
/************************** Function Declaration ******************************/
/******************************************************************************/

int32_t CN0411_DAC_set_value(struct cn0411_device *cn0411_dev,
			     float output_val);
int32_t CN0411_ADC_operation_mode (struct cn0411_device *cn0411_dev,
				   enum op_mode mode);
int32_t CN0411_ADC_setup (struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_set_ch (struct cn0411_device *cn0411_dev, uint8_t channel,
			   uint8_t ch_en);
int32_t CN0411_ADC_set_io1 (struct cn0411_device *cn0411_dev, uint8_t ch_gain);
int32_t CN0411_ADC_set_io2 (struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_conv_init(struct cn0411_device *cn0411_dev,
			     uint8_t conv_mod);
int32_t CN0411_ADC_read_ch(struct cn0411_device *cn0411_dev, uint8_t ch);
int32_t CN0411_read_temp(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_vpp(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_vdac(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_R20S(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_R200S(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_rdres(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_cond(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_off_res(struct cn0411_device *cn0411_dev);
int32_t CN0411_compensate_cond(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_tds(struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_int_calibrate(struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_sys_calibrate(struct cn0411_device *cn0411_dev);
int32_t CN0411_premeasurement(struct cn0411_device *cn0411_dev);
int32_t CN0411_set_gain_res(struct cn0411_device *cn0411_dev, int8_t ch_gain);
cmd_func CN0411_find_command(char *cmd);
void CN0411_cmd_process(struct cn0411_device *cn0411_dev);
int32_t CN0411_cmd_prompt(void);
uint8_t *CN0326_find_argv(uint8_t *args);
void CN0326_get_argv(char *dst, uint8_t *args);
void CN0411_cmd_help(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_sys_calib(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_off_res(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_conv_mode(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_autoset(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_set_dac(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_gain_res(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_rtd_val(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_pwm_freq(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_cell_const(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_solution(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_temp(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_vinput(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_read_dac(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_read_20s(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_read_200s(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_rdres(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_cond(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_cmd_tds(uint8_t *args, struct cn0411_device *cn0411_dev);
void CN0411_interrupt(void);
void CN0411_pwm_freq(uint16_t freq);
void CN0411_pwm_gen(void);
int32_t CN0411_init(struct cn0411_device *cn0411_dev,
		    struct cn0411_init_params cn0411_init_params);

#endif /* _CN0411_H_ */
