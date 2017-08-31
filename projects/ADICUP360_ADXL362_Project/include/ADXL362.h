/*
 * ADXL362.h
 *
 *  Created on: Jul 19, 2016
 *      Author: aravisha
 */

/**
******************************************************************************
*   @file     ADXL362.h
*   @brief    Header file for ADXL362 accelerometer control.
*   @version  V0.3
*   @author   ADI
*   @date     March 2016
*  @par Revision History:
*  - V0.1, September 2015: initial version.
*  - V0.2, October 2015: moved ACC definitions and added revision history.
*  - V0.3, March 2016: updated temperature parameters - name and value.
*
*******************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
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
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
**/
#ifndef ADXL362_H_
#define ADXL362_H_


/********************************* Definitions ********************************/

/* Accelerometer registers addresses */
#define STATUS_REG         0x0B
#define XDATA_L_REG        0x0E
#define YDATA_L_REG        0x10
#define ZDATA_L_REG        0x12
#define TEMP_L_REG         0x14
#define SOFT_RESET_REG     0x1F
#define THRESH_ACT_L       0x20
#define THRESH_ACT_H       0x21
#define TIME_ACT           0x22
#define THRESH_INACT_L     0x23
#define THRESH_INACT_H     0x24
#define TIME_INACT_L       0x25
#define TIME_INACT_H       0x26
#define ACT_INACT_CTL      0x27
#define INTMAP1            0x2A
#define INTMAP2            0x2B
#define POWER_CTL_REG      0x2D


/**************************** Configuration parameters **********************/

#define TEMP_ADC        1     /* Temperature display units: 1 for ADC, 0 for degrees Celsius */

/* Temperature parameters */
#define ACC_TEMP_BIAS             (float)350         /* Accelerometer temperature bias(in ADC codes) at 25 Deg C */
#define ACC_TEMP_SENSITIVITY      (float)0.065       /* Accelerometer temperature sensitivity  from datasheet (DegC per Code) */

#define SCAN_SENSOR_TIME   500    /* Accelerometer scan interval in ms */

#define ACT_VALUE          50     /* Activity threshold value */

#define INACT_VALUE        50     /* Inactivity threshold value */

#define ACT_TIMER          100    /* Activity timer value in ms */

#define INACT_TIMER        10     /* Inactivity timer value in seconds */


/****************************** Global Data ***********************************/

extern int16_t i16SensorX;
extern int16_t i16SensorY;
extern int16_t i16SensorZ;
extern int16_t i16SensorT;

extern volatile uint32_t ui32timer_counter;


/*************************** Functions prototypes *****************************/
void ADXL362_Init(void);
void Sensor_Init(void);
void Sensor_Start(void);
void Sensor_Stop(void);
void Sensor_Scan(void);
uint8_t Sensor_Delay(uint8_t ui8StartFlag, uint32_t *pu32EndTm, uint32_t ui32Delay);

#endif /* ADXL362_H_ */

/* End Of File */
