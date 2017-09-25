/*!
 *****************************************************************************
 * @file:    AD7988.h
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2017 Analog Devices, Inc.

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

#ifndef AD7988_H_
#define AD7988_H_

/********************************* Global data ********************************/
#define ADC_VREF  4.096 // V

/************************** Variable Definitions ******************************/
typedef enum{
   AD7988_DISABLE,
   AD7988_ENABLE
}enAD7988Status;

typedef enum{              /* A2,   A1,   A0 */
   AD7988_RS_RAMGE_1,      /* 0,    0,    0  */
   AD7988_RS_RAMGE_2,      /* 0,    0,    1  */
   AD7988_RS_RAMGE_3,      /* 0,    1,    0  */
   AD7988_RS_RAMGE_4,      /* 0,    1,    1  */
   AD7988_RS_RAMGE_5,      /* 1,    0,    0  */
   AD7988_RH_MODE
}enAD7988OpMode;

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Initializes the AD7988 */
void AD7988_Init(void);
/* Selects the desired gain resistor R1 to R5 */
void AD7988_SensorRangeSelect(enAD7988OpMode mode);
/* Selects the desired mode of operation by enabling or disabling the power on the correct pins */
void AD7988_SetOperationMode(enAD7988OpMode mode);
/* Reads AD7988 data from the SPI */
void AD7988_ReadData(uint16_t *data);
/* Converts the read ADC data to the corresponding voltage */
float AD7988_DataToVoltage(uint16_t ui16Adcdata);

#endif //AD7988_H_
