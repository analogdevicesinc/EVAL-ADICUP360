/*!
 *****************************************************************************
 * @file:    ADXL355.h
 * @brief:   ADXL355 accelerometer IC
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2016-2017 Analog Devices, Inc.

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

#ifndef ADXL355_H_
#define ADXL355_H_

/********************************* Definitions ********************************/

/* ADXL355 registers addresses */
#define DEVID_AD                 0x00
#define DEVID_MST                0x01
#define PARTID                   0x02
#define REVID                    0x03
#define STATUS                   0x04
#define FIFO_ENTRIES             0x05
#define TEMP2                    0x06
#define TEMP1                    0x07
#define XDATA3                   0x08
#define XDATA2                   0x09
#define XDATA1                   0x0A
#define YDATA3                   0x0B
#define YDATA2                   0x0C
#define YDATA1                   0x0D
#define ZDATA3                   0x0E
#define ZDATA2                   0x0F
#define ZDATA1                   0x10
#define FIFO_DATA                0x11
#define OFFSET_X_H               0x1E
#define OFFSET_X_L               0x1F
#define OFFSET_Y_H               0x20
#define OFFSET_Y_L               0x21
#define OFFSET_Z_H               0x22
#define OFFSET_Z_L               0x23
#define ACT_EN                   0x24
#define ACT_THRESH_H             0x25
#define ACT_THRESH_L             0x26
#define ACT_COUNT                0x27
#define FILTER                   0x28
#define FIFO_SAMPLES             0x29
#define INT_MAP                  0x2A
#define SYNC                     0x2B
#define RANGE                    0x2C
#define POWER_CTL                0x2D
#define SELF_TEST                0x2E
#define RESET                    0x2F

/**************************** Configuration parameters **********************/

/* Temperature parameters */
#define ADXL355_TEMP_BIAS       (float)1852.0      /* Accelerometer temperature bias(in ADC codes) at 25 Deg C */
#define ADXL355_TEMP_SLOPE      (float)-9.05       /* Accelerometer temperature change from datasheet (LSB/degC) */

/* Accelerometer parameters */
#define ADXL_RANGE     2     /* ADXL362 sensitivity: 2, 4, 8 [g] */

#define ACT_VALUE          50     /* Activity threshold value */

#define INACT_VALUE        50     /* Inactivity threshold value */

#define ACT_TIMER          100    /* Activity timer value in ms */

#define INACT_TIMER        10     /* Inactivity timer value in seconds */

/****************************** Global Data ***********************************/

extern volatile int32_t i32SensorX;
extern volatile int32_t i32SensorY;
extern volatile int32_t i32SensorZ;

extern volatile uint32_t ui32SensorX;
extern volatile uint32_t ui32SensorY;
extern volatile uint32_t ui32SensorZ;
extern volatile uint32_t ui32SensorT;

extern volatile uint32_t ui32timer_counter;


/*************************** Functions prototypes *****************************/

void ADXL355_Init(void);
void ADXL355_Start_Sensor(void);
void ADXL355_Stop_Sensor(void);
void ADXL355_Data_Scan(void);
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData);

#endif /* ADXL355_H_ */
