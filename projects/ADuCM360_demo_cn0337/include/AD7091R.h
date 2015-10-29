/**
******************************************************************************
*   @file     AD7091R.h
*   @brief    Header file for AD7091R converter
*   @version  V0.1
*   @author   ADI
*   @date     October 2015
*  @par Revision History:
*  - V0.1, October 2015: initial version.
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
#ifndef AD7091R_H_
#define AD7091R_H_


/****************************** Global Data ***********************************/

extern uint8_t u8StartCounter;                           /* Declaration of start counter variable */


/****************************** Internal types *********************************/
/* Converter operation modes */
typedef enum {
   NORMAL = 1,
   POWER_DOWN
} enOpModes;


/*************************** Functions prototypes *****************************/

void AD7091R_Init(void);
uint16_t AD7091R_Scan(void);
void AD7091R_PowerUp(void);
float AD7091R_ConvertToVolts(uint16_t u16adcValue, float f32VRef);


/********************************* Internal defines ********************************/

#define ADC_RESOLUTION       (1 << 12)          /* ADC resolution = 12 bits */


/**************************** Configuration parameters **********************/

#define AD7091R_OPERATION_MODE      POWER_DOWN           /* Select converter operation mode */

#define SCAN_TIME                500               /* Select how often to read ADC conversion result -> in [msec] */

#define VREF                     2.5            /* Select reference voltage -> in [V] */


#endif /* AD7091R_H_ */


