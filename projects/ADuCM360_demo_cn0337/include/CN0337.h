/*!
 *****************************************************************************
 * @file:    CN0337.h
 * @brief:   CN0337 control
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

#ifndef CN0337_H_
#define CN0337_H_


/****************************** Global Data ***********************************/

extern const float        C_rtd[];


/*************************** Functions prototypes *****************************/

void CN0337_Init(void);
void CN0337_Interrupt(void);
void CN0337_WriteData(float f32temp, float f32r, float f32voltage, uint16_t u16adc );
void CN0337_Printf(const char *fmt, ...);
float CN0337_CalculateTemp(float r);
float CN0337_CalculateResistance(uint16_t u16adc, float f32voltage);


/******************************* Internal defines ******************************/

#define  _CR_KEY              13                 /* ASCII code for ENTER (CR) */

#define  INVALID_DATA         0x13               /*Invalid value */
#define  VALUE_TO_SMALL       0x14                  /* Indexer for values under range */
#define  VALUE_TO_BIG         0x15                  /* Indexer for values over range */

#define TWO_POINT_CALIBRATION   2          /* Use two point calibration method */
#define TRANSFER_FUNCTION       3                 /* Use basic transfer function of the circuit */

/**************************** Configuration parameters **********************/

#define  TMIN           (0)              /* Tmin [˚C] */
#define  TMAX           (300)            /* Tmax [˚C] */
#define  RMIN           (100)            /* Resistance [Ohms] at Tmin */
#define  RMAX           (212.052)        /* Resistance [Ohms] at Tmax  */
#define  NSEG            100             /* Number of sections in look-up table */
#define  RSEG            1.12052         /* (RMAX-RMIN)/NSEG = resistance RSEG [Ohms] of each segment */

#define RTD_FORMULA     TRANSFER_FUNCTION      /* Select which method do you want for RTD resistance calculation: TRANSFER_FUNCTION or TWO_POINT_CALIBRATION */

#if(RTD_FORMULA == TWO_POINT_CALIBRATION)           /* Set ADC max and min values only if you choose to use two point calibration */

#define  ADC_MIN             152              /* ADC min for RMIN */
#define  ADC_MAX            4095            /* ADC max for RMAX */

#endif

#endif /* CN0337_H_ */
