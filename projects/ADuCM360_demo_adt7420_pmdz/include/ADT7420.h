/*!
 *****************************************************************************
 * @file:    ADT7420.h
 * @brief:   ADT7420 temperature IC
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

#ifndef ADT7420_H_
#define ADT7420_H_

/********************************* Definitions ********************************/

/* ADT7420 registers addresses */
#define TEMP_MSB_REG                0x00
#define TEMP_LSB_REG                0x01
#define STATUS_REG                  0x02
#define CONFIG_REG                  0x03
#define T_HIGH_SETPOINT_MSB_REG     0x04
#define T_HIGH_SETPOINT_LSB_REG     0x05
#define T_LOW_SETPOINT_MSB_REG      0x06
#define T_LOW_SETPOINT_LSB_REG      0x07
#define T_CRIT_SETPOINT_MSB_REG     0x08
#define T_CRIT_SETPOINT_LSB_REG     0x09
#define T_HYST_SETPOINT_REG         0x0A
#define ID_REG                      0x0B
#define SOFTWARE_RESET              0x2F

/*******************************************************************************
************************* Configurations definitions ***************************
********************************************************************************/

/* ADT7420 I2C Address */
#define ADT7420_ADDRESS    0x48              /* ADT7420 I2C Address with A1 and A0 pin tied low */

/**************************** Configuration Parameters **********************/

#define HIGH                           1
#define LOW                            0

/* ADT7420 Setup parameters */
#define FAULT_TRIGGER_1                0x00
#define FAULT_TRIGGER_2                0x01
#define FAULT_TRIGGER_3                0x02
#define FAULT_TRIGGER_4                0x03

#define CT_PIN_POLARITY                (LOW << 2)
#define INT_PIN_POLARITY               (LOW << 3)
#define INT_CT_MODE                    (HIGH << 4)

#define CONTINUOUS_CONVERSION_MODE     0x00
#define ONE_SHOT_MODE                  0x20
#define ONE_SAMPLE_PER_SECOND_MODE     0x40
#define SHUTDOWN_MODE                  0x60

#define RESOLUTION_13_BITS             0x00
#define RESOLUTION_16_BITS             0x80

/* Temperature monitoring parameters */
#define TEMP_HIGH_SETPOINT             75          /* Default value of the ADT7420 */
#define TEMP_LOW_SETPOINT              0          /* Default value of the ADT7420 */
#define TEMP_CRITICAL_SETPOINT         100         /* Default value of the ADT7420 */
#define TEMP_HYSTERSIS_SETPOINT        5           /* Default value of the ADT7420 */

/****************************** Global Data ***********************************/

extern uint8_t ui8configAdt7420;

/*************************** Functions Declarations *****************************/

void ADT7420_Init(void);
uint16_t ADT7420_Read_Temp(void);
uint16_t ADT7420_Read_One_Reg (uint8_t ui8regAddress);
uint16_t ADT7420_Read_Two_Reg (uint8_t ui8regAddress);
void ADT7420_Write_One_Reg (uint8_t ui8regAddress, uint8_t ui8Data);
void ADT7420_Write_Two_Reg (uint8_t ui8regAddress, uint8_t ui8Data, uint8_t ui8Data2);
uint16_t ADT7420_Convert_Degrees_To_Hex (int16_t i16degrees);
float ADT7420_Convert_Hex_To_Degrees (uint16_t ui16tempResults);
void ADT7420_Power_Down ();
void ADT7420_Power_Up ();


#endif /* ADT7420_H_ */
