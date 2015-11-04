/**
******************************************************************************
*   @file     CN0336.h
*   @brief    Header file for CN0336 control.
*   @version  V0.1
*   @author   ADI
*   @date     November 2015
*  @par Revision History:
*  - V0.1, November 2015: initial version.
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
#ifndef CN0336_H_
#define CN0336_H_


/*************************** Functions prototypes *****************************/

void CN0336_Init(void);
void CN0336_Interrupt(void);
void CN0336_WriteData(float f32current, float f32voltage, uint16_t u16adc);
float CN0336_InputCurrent(uint16_t u16adc, float f32voltage);

/******************************* Internal defines ******************************/

#define  _CR_KEY              13                 /* ASCII code for ENTER (CR) */


#define  INVALID_DATA         0x13               /*Invalid value */
#define  VALUE_TO_SMALL       0x14                  /* Indexer for values under range */
#define  VALUE_TO_BIG         0x15                  /* Indexer for values over range */

#define TWO_POINT_CALIBRATION   2          /* Use two point calibration method */
#define TRANSFER_FUNCTION       3                 /* Use basic transfer function of the circuit */

#define V_OFFSET              0.1                                     /* Voltage offset of the circuit from data sheet -> [V] */
#define MULT_FACTOR           1000

/**************************** Configuration parameters **********************/

#define  IMIN           4             /* Imin [mA] */
#define  IMAX           20            /* Imax [mA] */

#define CALC_FORMULA     TRANSFER_FUNCTION      /* Select which method do you want for RTD resistance calculation: TRANSFER_FUNCTION or TWO_POINT_CALIBRATION */

#if(CALC_FORMULA == TWO_POINT_CALIBRATION)           /* Set ADC max and min values only if you choose to use two point calibration */

#define  ADC_MIN             163              /* ADC min for IMIN */
#define  ADC_MAX            3932              /* ADC max for IMAX */

#endif

#endif /* CN0336_H_ */
