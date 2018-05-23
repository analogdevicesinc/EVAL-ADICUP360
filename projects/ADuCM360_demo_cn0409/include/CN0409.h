/**
******************************************************************************
*   @file     CN0409.h
*   @brief    Header file for CN0409 application
*   @version  V0.1
*   @author   ADI
*   @date     May 2017
*   @par Revision History:
*  - V0.1, May 2017: initial version.
*
*
*******************************************************************************
* Copyright 2017(c) Analog Devices, Inc.
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
*********************************************************************************/
#ifndef CN0409_H_
#define CN0409_H_

#include "stdint-gcc.h"

#define RATIO_COEF_FLASH     0x1f000ul
#define NONRATIO_COEF_FLASH  0x1f200ul
#define GLI_COEF_FLASH       0x1f400ul

#define CALIBRATION_AFE_VALUE 12270

/*Calibration solutions values*/
#define FIRST_POINT 0.1
#define SECOND_POINT 40.0
#define THIRD_POINT 1000.0

#define AVERAGE_DATA 256.0

#define STRINGIFY(arg)  #arg
#define STRINGIFY1(arg) STRINGIFY(arg)

/***************************** Include Files **********************************/
extern float fpTurbidity;

/****************************** Variables ***********************************/
enum CN0409_CoeffUpdate {
	UPDATED = 0x05,
	NOT_UPDATED
};

struct CN0409_ISORatioCalibrationCoeff {
	float slope;
	float intercept;
	enum CN0409_CoeffUpdate status;
};

struct CN0409_ISOCalibrationCoeff {
	float m;
	float b;
	enum CN0409_CoeffUpdate status;
};

struct CN0409_NtuSolutions {
	float isoRatio1;
	float isoRatio2;
	float Gli1;
	float Gli2;
};

struct CN0409_GLICoeff {
	float calSlope;
	float cal0;
	enum CN0409_CoeffUpdate status;
};

enum CN0409_Type {
	RATIOMETRIC = 0,
	NONRATIOMETRIC,
	GLI
};

/*************************** Functions prototypes *****************************/
void CN0409_Init(void);
void CN0409_CalculateRawData(void);
void CN0409_InteractiveMenu(void);
void CN0409_TurbidityCalculation(void);
void CN0409_CalculateCoeff(enum CN0409_Type mode);
void CN0409_ChannelOffsetCalibration(void);
void CN0409_AFECalibration(void);

#endif /* CN0409_H_ */
