/*!
 *****************************************************************************
 * @file:    AD7793.h
 * @brief:   AD7793 converter
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

#ifndef AD7793_H_
#define AD7793_H_


/****************************** Global Data ***********************************/

/*Operation mode*/
typedef enum {
   CONTINUOUS_CONV = 0,
   SINGLE_CONV,
   IDLE_MODE,
   POWER_DOWN_MODE,
   CAL_INT_ZERO_MODE,
   CAL_INT_FULL_MODE,
   CAL_SYS_ZERO_MODE,
   CAL_SYS_FULL_MODE
} enMode;

/*************************** Functions prototypes *****************************/

void AD7793_Init(void);
void AD7793_Reset(void);
uint32_t AD7793_ReadRegister( uint8_t ui8address);
void AD7793_WriteRegister( uint8_t ui8address, uint32_t ui32data);
void AD7793_SelectChannel(uint8_t ui8channel);
uint32_t AD7793_Scan(enMode mode,  uint8_t channel);
void AD7793_Calibrate(uint8_t ui8channel, enMode mode);
float AD7793_ConvertToVolts(uint32_t u32adcValue);

/********************************* Internal defines ********************************/

/* AD7793 Registers */
#define AD7793_REG_COMM       0 /* Communications Register(WO, 8-bit) */
#define AD7793_REG_STAT       0 /* Status Register       (RO, 8-bit) */
#define AD7793_REG_MODE       1 /* Mode Register        (RW, 16-bit */
#define AD7793_REG_CONF       2 /* Configuration Register (RW, 16-bit)*/
#define AD7793_REG_DATA       3 /* Data Register        (RO, 24-bit) */
#define AD7793_REG_ID         4 /* ID Register       (RO, 8-bit) */
#define AD7793_REG_IO         5 /* IO Register       (RW, 8-bit) */
#define AD7793_REG_OFFSET     6 /* Offset Register       (RW, 24-bit) */
#define AD7793_REG_FULLSCALE   7 /* Full-Scale Register   (RW, 24-bit) */

/* Communications Register Bit Designations (AD7793_REG_COMM) */
#define AD7793_COMM_WRITE  (0 << 6)          /* Write Operation */
#define AD7793_COMM_READ    (1 << 6)         /* Read Operation */
#define AD7793_COMM_ADR(x)   (((x) & 0x07) << 3)

#define AD7793_MODE_MSK    (~(0x07 << 13))
#define AD7793_CONF_MSK     (~(0x07))

/* RDY bit AD7793 STATUS reg - b7*/
#define RDY_BIT       0x80

/* AD7793_CONF options */
#define AD7793_GAIN_1       0   /* input range = 2.5V */
#define AD7793_GAIN_2       1
#define AD7793_GAIN_4       2
#define AD7793_GAIN_8       3
#define AD7793_GAIN_16      4
#define AD7793_GAIN_32      5
#define AD7793_GAIN_64      6
#define AD7793_GAIN_128     7
#define AD7793_REFSEL      1 << 7
#define AD7793_BUF         1 << 4

/* AD7793 channels*/
#define AD7793_CH_AIN1P_AIN1M 0 /* AIN1(+) - AIN1(-) */
#define AD7793_CH_AIN2P_AIN2M 1 /* AIN2(+) - AIN2(-) */
#define AD7793_CH_AIN3P_AIN3M 2 /* AIN3(+) - AIN3(-) */
#define AD7793_CH_AIN1M_AIN1M 3 /* AIN1(-) - AIN1(-) */
#define AD7793_CH_TEMP        6 /* Temp Sensor */
#define AD7793_CH_AVDD_MONITOR 7 /* AVDD Monitor */

/**************************** Configuration parameters **********************/
#define AD7793_GAIN              AD7793_GAIN_1


#endif /* AD7793_H_ */


