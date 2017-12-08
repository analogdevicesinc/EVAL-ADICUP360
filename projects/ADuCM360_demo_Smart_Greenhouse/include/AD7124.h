/*!
 *****************************************************************************
 * @file:    AD7124.h
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

#ifndef _AD7124_H
   #define _AD7124_H

   #include <stdint.h>

   enum ad7124_registers
   {
      AD7124_Status = 0x00,
      AD7124_ADC_Control,
      AD7124_Data,
      AD7124_IOCon1,
      AD7124_IOCon2,
      AD7124_ID,
      AD7124_Error,
      AD7124_Error_En,
      AD7124_Mclk_Count,
      AD7124_Channel_0,
      AD7124_Channel_1,
      AD7124_Channel_2,
      AD7124_Channel_3,
      AD7124_Channel_4,
      AD7124_Channel_5,
      AD7124_Channel_6,
      AD7124_Channel_7,
      AD7124_Channel_8,
      AD7124_Channel_9,
      AD7124_Channel_10,
      AD7124_Channel_11,
      AD7124_Channel_12,
      AD7124_Channel_13,
      AD7124_Channel_14,
      AD7124_Channel_15,
      AD7124_Config_0,
      AD7124_Config_1,
      AD7124_Config_2,
      AD7124_Config_3,
      AD7124_Config_4,
      AD7124_Config_5,
      AD7124_Config_6,
      AD7124_Config_7,
      AD7124_Filter_0,
      AD7124_Filter_1,
      AD7124_Filter_2,
      AD7124_Filter_3,
      AD7124_Filter_4,
      AD7124_Filter_5,
      AD7124_Filter_6,
      AD7124_Filter_7,
      AD7124_Offset_0,
      AD7124_Offset_1,
      AD7124_Offset_2,
      AD7124_Offset_3,
      AD7124_Offset_4,
      AD7124_Offset_5,
      AD7124_Offset_6,
      AD7124_Offset_7,
      AD7124_Gain_0,
      AD7124_Gain_1,
      AD7124_Gain_2,
      AD7124_Gain_3,
      AD7124_Gain_4,
      AD7124_Gain_5,
      AD7124_Gain_6,
      AD7124_Gain_7,
      AD7124_REG_NO
   };

   enum ad7124_reg_access
   {
        AD7124_RW = 1,   /* Read and Write */
        AD7124_R  = 2,   /* Read only */
        AD7124_W  = 3,   /* Write only */
   };

   typedef struct _ad7124_st_reg
   {
      int32_t addr;
      int32_t value;
      int32_t size;
      int32_t rw;
   } ad7124_st_reg;

   enum AD7124_reg_map
   {
      COMM_REG     = 0x00,
      STATUS_REG   = 0x00,
      ADC_CTRL_REG = 0x01,
      DATA_REG     = 0x02,
      IO_CTRL1_REG = 0x03,
      IO_CTRL2_REG = 0x04,
      ID_REG       = 0x05,
      ERR_REG      = 0x06,
      ERREN_REG    = 0x07,
      CH0_MAP_REG  = 0x09,
      CH1_MAP_REG  = 0x0A,
      CH2_MAP_REG  = 0x0B,
      CH3_MAP_REG  = 0x0C,
      CH4_MAP_REG  = 0x0D,
      CH5_MAP_REG  = 0x0E,
      CH6_MAP_REG  = 0x0F,
      CH7_MAP_REG  = 0x10,
      CH8_MAP_REG  = 0x11,
      CH9_MAP_REG  = 0x12,
      CH10_MAP_REG = 0x13,
      CH11_MAP_REG = 0x14,
      CH12_MAP_REG = 0x15,
      CH13_MAP_REG = 0x16,
      CH14_MAP_REG = 0x17,
      CH15_MAP_REG = 0x18,
      CFG0_REG     = 0x19,
      CFG1_REG     = 0x1A,
      CFG2_REG     = 0x1B,
      CFG3_REG     = 0x1C,
      CFG4_REG     = 0x1D,
      CFG5_REG     = 0x1E,
      CFG6_REG     = 0x1F,
      CFG7_REG     = 0x20,
      FILT0_REG    = 0x21,
      FILT1_REG    = 0x22,
      FILT2_REG    = 0x23,
      FILT3_REG    = 0x24,
      FILT4_REG    = 0x25,
      FILT5_REG    = 0x26,
      FILT6_REG    = 0x27,
      FILT7_REG    = 0x28,
      OFFS0_REG    = 0x29,
      OFFS1_REG    = 0x2A,
      OFFS2_REG    = 0x2B,
      OFFS3_REG    = 0x2C,
      OFFS4_REG    = 0x2D,
      OFFS5_REG    = 0x2E,
      OFFS6_REG    = 0x2F,
      OFFS7_REG    = 0x30,
      GAIN0_REG    = 0x31,
      GAIN1_REG    = 0x32,
      GAIN2_REG    = 0x33,
      GAIN3_REG    = 0x34,
      GAIN4_REG    = 0x35,
      GAIN5_REG    = 0x36,
      GAIN6_REG    = 0x37,
      GAIN7_REG    = 0x38,
   };

   /* Communication Register bits */
   #define AD7124_COMM_REG_WEN    (0 << 7)
   #define AD7124_COMM_REG_WR     (0 << 6)
   #define AD7124_COMM_REG_RD     (1 << 6)
   #define AD7124_COMM_REG_RA(x)  ((x) & 0x3F)

   /* Status Register bits */
   #define AD7124_STATUS_REG_RDY          (1 << 7)
   #define AD7124_STATUS_REG_ERROR_FLAG   (1 << 6)
   #define AD7124_STATUS_REG_POR_FLAG     (1 << 4)
   #define AD7124_STATUS_REG_CH_ACTIVE(x) ((x) & 0xF)

   /* ADC_Control Register bits */
   #define AD7124_ADC_CTRL_REG_DOUT_RDY_DEL   (1 << 12)
   #define AD7124_ADC_CTRL_REG_CONT_READ      (1 << 11)
   #define AD7124_ADC_CTRL_REG_DATA_STATUS    (1 << 10)
   #define AD7124_ADC_CTRL_REG_CS_EN          (1 << 9)
   #define AD7124_ADC_CTRL_REG_REF_EN         (1 << 8)
   #define AD7124_ADC_CTRL_REG_POWER_MODE(x)  (((x) & 0x3) << 6)
   #define AD7124_ADC_CTRL_REG_MODE(x)        (((x) & 0xF) << 2)
   #define AD7124_ADC_CTRL_REG_CLK_SEL(x)    (((x) & 0x3) << 0)

   /* IO_Control_1 Register bits */
   #define AD7124_IO_CTRL1_REG_GPIO_DAT2     (1 << 23)
   #define AD7124_IO_CTRL1_REG_GPIO_DAT1     (1 << 22)
   #define AD7124_IO_CTRL1_REG_GPIO_CTRL2    (1 << 19)
   #define AD7124_IO_CTRL1_REG_GPIO_CTRL1    (1 << 18)
   #define AD7124_IO_CTRL1_REG_PDSW          (1 << 15)
   #define AD7124_IO_CTRL1_REG_IOUT1(x)      (((x) & 0x7) << 11)
   #define AD7124_IO_CTRL1_REG_IOUT0(x)      (((x) & 0x7) << 8)
   #define AD7124_IO_CTRL1_REG_IOUT_CH1(x)   (((x) & 0xF) << 4)
   #define AD7124_IO_CTRL1_REG_IOUT_CH0(x)   (((x) & 0xF) << 0)

   /*IO_Control_1 AD7124-8 specific bits */
   #define AD7124_8_IO_CTRL1_REG_GPIO_DAT4     (1 << 23)
   #define AD7124_8_IO_CTRL1_REG_GPIO_DAT3     (1 << 22)
   #define AD7124_8_IO_CTRL1_REG_GPIO_DAT2     (1 << 21)
   #define AD7124_8_IO_CTRL1_REG_GPIO_DAT1     (1 << 20)
   #define AD7124_8_IO_CTRL1_REG_GPIO_CTRL4    (1 << 19)
   #define AD7124_8_IO_CTRL1_REG_GPIO_CTRL3    (1 << 18)
   #define AD7124_8_IO_CTRL1_REG_GPIO_CTRL2    (1 << 17)
   #define AD7124_8_IO_CTRL1_REG_GPIO_CTRL1    (1 << 16)

   /* IO_Control_2 Register bits */
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 15)
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 14)
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 11)
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 10)
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 5)
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 4)
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
   #define AD7124_IO_CTRL2_REG_GPIO_VBIAS0   (1)

   /*IO_Control_2 AD7124-8 specific bits */
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS15  (1 << 15)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS14  (1 << 14)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS13  (1 << 13)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS12  (1 << 12)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS11  (1 << 11)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS10  (1 << 10)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS9   (1 << 9)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS8   (1 << 8)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 7)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 6)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 5)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 4)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 3)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 2)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
   #define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS0   (1 << 0)

   /* ID Register bits */
   #define AD7124_ID_REG_DEVICE_ID(x)   (((x) & 0xF) << 4)
   #define AD7124_ID_REG_SILICON_REV(x) (((x) & 0xF) << 0)

   /* Error Register bits */
   #define AD7124_ERR_REG_LDO_CAP_ERR        (1 << 19)
   #define AD7124_ERR_REG_ADC_CAL_ERR        (1 << 18)
   #define AD7124_ERR_REG_ADC_CONV_ERR       (1 << 17)
   #define AD7124_ERR_REG_ADC_SAT_ERR        (1 << 16)
   #define AD7124_ERR_REG_AINP_OV_ERR        (1 << 15)
   #define AD7124_ERR_REG_AINP_UV_ERR        (1 << 14)
   #define AD7124_ERR_REG_AINM_OV_ERR        (1 << 13)
   #define AD7124_ERR_REG_AINM_UV_ERR        (1 << 12)
   #define AD7124_ERR_REG_REF_DET_ERR        (1 << 11)
   #define AD7124_ERR_REG_DLDO_PSM_ERR       (1 << 9)
   #define AD7124_ERR_REG_ALDO_PSM_ERR       (1 << 7)
   #define AD7124_ERR_REG_SPI_IGNORE_ERR     (1 << 6)
   #define AD7124_ERR_REG_SPI_SLCK_CNT_ERR   (1 << 5)
   #define AD7124_ERR_REG_SPI_READ_ERR       (1 << 4)
   #define AD7124_ERR_REG_SPI_WRITE_ERR      (1 << 3)
   #define AD7124_ERR_REG_SPI_CRC_ERR        (1 << 2)
   #define AD7124_ERR_REG_MM_CRC_ERR         (1 << 1)

   /* Error_En Register bits */
   #define AD7124_ERREN_REG_MCLK_CNT_EN           (1 << 22)
   #define AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN   (1 << 21)
   #define AD7124_ERREN_REG_LDO_CAP_CHK(x)        (((x) & 0x3) << 19)
   #define AD7124_ERREN_REG_ADC_CAL_ERR_EN        (1 << 18)
   #define AD7124_ERREN_REG_ADC_CONV_ERR_EN       (1 << 17)
   #define AD7124_ERREN_REG_ADC_SAT_ERR_EN        (1 << 16)
   #define AD7124_ERREN_REG_AINP_OV_ERR_EN        (1 << 15)
   #define AD7124_ERREN_REG_AINP_UV_ERR_EN        (1 << 14)
   #define AD7124_ERREN_REG_AINM_OV_ERR_EN        (1 << 13)
   #define AD7124_ERREN_REG_AINM_UV_ERR_EN        (1 << 12)
   #define AD7124_ERREN_REG_REF_DET_ERR_EN        (1 << 11)
   #define AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN (1 << 10)
   #define AD7124_ERREN_REG_DLDO_PSM_ERR_ERR      (1 << 9)
   #define AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN (1 << 8)
   #define AD7124_ERREN_REG_ALDO_PSM_ERR_EN       (1 << 7)
   #define AD7124_ERREN_REG_SPI_IGNORE_ERR_EN     (1 << 6)
   #define AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN   (1 << 5)
   #define AD7124_ERREN_REG_SPI_READ_ERR_EN       (1 << 4)
   #define AD7124_ERREN_REG_SPI_WRITE_ERR_EN      (1 << 3)
   #define AD7124_ERREN_REG_SPI_CRC_ERR_EN        (1 << 2)
   #define AD7124_ERREN_REG_MM_CRC_ERR_EN         (1 << 1)

   /* Channel Registers 0-15 bits */
   #define AD7124_CH_MAP_REG_CH_ENABLE    (1 << 15)
   #define AD7124_CH_MAP_REG_SETUP(x)     (((x) & 0x7) << 12)
   #define AD7124_CH_MAP_REG_AINP(x)      (((x) & 0x1F) << 5)
   #define AD7124_CH_MAP_REG_AINM(x)      (((x) & 0x1F) << 0)

   /* Configuration Registers 0-7 bits */
   #define AD7124_CFG_REG_BIPOLAR     (1 << 11)
   #define AD7124_CFG_REG_BURNOUT(x)  (((x) & 0x3) << 9)
   #define AD7124_CFG_REG_REF_BUFP    (1 << 8)
   #define AD7124_CFG_REG_REF_BUFM    (1 << 7)
   #define AD7124_CFG_REG_AIN_BUFP    (1 << 6)
   #define AD7124_CFG_REG_AINN_BUFM   (1 << 5)
   #define AD7124_CFG_REG_REF_SEL(x)  ((x) & 0x3) << 3
   #define AD7124_CFG_REG_PGA(x)      (((x) & 0x7) << 0)

   /* Filter Register 0-7 bits */
   #define AD7124_FILT_REG_FILTER(x)        ((uint32_t)((x) & 0x7) << 21)
   #define AD7124_FILT_REG_REJ60             ((uint32_t)1 << 20)
   #define AD7124_FILT_REG_POST_FILTER(x)    ((uint32_t)((x) & 0x7) << 17)
   #define AD7124_FILT_REG_SINGLE_CYCLE      ((uint32_t)1 << 16)
   #define AD7124_FILT_REG_FS(x)             ((uint32_t)((x) & 0x7FF) << 0)

   extern int32_t AD7124_Reset(void);

   /* Reads and returns the value of a device register. */
   extern uint32_t AD7124_ReadDeviceRegister(enum ad7124_registers reg);

   /* Writes the specified value to a device register. */
   extern int32_t AD7124_WriteDeviceRegister(enum ad7124_registers reg, uint32_t value);

   /* Reads the value of the specified register. */
   extern int32_t AD7124_ReadRegister(ad7124_st_reg* pReg);

   /* Writes the value of the specified register. */
   extern int32_t AD7124_WriteRegister(ad7124_st_reg reg);

   /* Reads the value of the specified register without a device state check. */
   extern int32_t AD7124_NoCheckReadRegister(ad7124_st_reg* pReg);

   /* Writes the value of the specified register without a device state check. */
   extern int32_t AD7124_NoCheckWriteRegister(ad7124_st_reg reg);

   /* Waits until the device can accept read and write user actions. */
   extern int32_t AD7124_WaitForSpiReady(uint32_t timeout);

   /* Waits until a new conversion result is available. */
   extern int32_t AD7124_WaitForConvReady(uint32_t timeout);

   /* Reads the conversion result from the device. */
   extern int32_t AD7124_ReadData(int32_t* pData);

   /* Computes the CRC checksum for a data buffer. */
   extern uint8_t AD7124_ComputeCRC8(uint8_t* pBuf, uint8_t bufSize);

   /* Updates the device SPI interface settings. */
   extern void AD7124_UpdateDevSpiSettings(void);

   /* Initializes the AD7124. */
   extern int32_t AD7124_Setup(void);

   extern ad7124_st_reg ad7124_regs[57];
   extern ad7124_st_reg *regs;

   static const uint8_t _SPI_MODE = 0x03;
   static const uint8_t _RESET = 0xFF;
   static const uint8_t _DUMMY_BYTE = 0xFF;
   static const uint16_t _READ_FLAG = 0x4000;
   static const uint8_t _DELAY_TIMING = 0x02;

   #define AD7124_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
   #define AD7124_DISABLE_CRC 0
   #define AD7124_USE_CRC 1
   #define AD7124_READ_DATA 2

   #define INVALID_VAL -1 /* Invalid argument */
   #define COMM_ERR    -2 /* Communication error on receive */
   #define TIMEOUT     -3 /* A timeout has occured */

#endif
