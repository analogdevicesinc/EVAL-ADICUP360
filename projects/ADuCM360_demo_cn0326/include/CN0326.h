/**
******************************************************************************
*   @file     CN0326.h
*   @brief    Header file for CN0326 control.
*   @version  V0.1
*   @author   ADI
*   @date     December 2015
*  @par Revision History:
*  - V0.1, December 2015: initial version.
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
#ifndef CN0326_H_
#define CN0326_H_

/****************************** Internal types *********************************/

typedef  void (*cmdFunc)(uint8_t *);

/*************************** Functions prototypes *****************************/

void CN0326_Init(void);
void CN0326_Interrupt(void);
float CN0326_CalculateTemp(void);
float CN0326_CalculatePH(void);
void CN0326_CmdHelp(uint8_t *args);
void CN0326_CmdTemp(uint8_t *args);
void CN0326_CmdPH(uint8_t *args);
void CN0326_CmdCalibration(uint8_t *args);
void CN0326_CmdReset(uint8_t *args);
cmdFunc CN0326_FindCommand(char *cmd);
void CN0326_CmdProcess(void);
int CN0326_CmdPrompt(void);
uint8_t *CN0326_FindArgv(uint8_t *args);
void CN0326_GetArgv(char *dst, uint8_t *args);

/******************************* Internal defines ******************************/

#define YES    1
#define NO     2

#define TEMP_COEFF           0.00385        /* [Ω/Ω/˚C] - defined by the standard => check the documentation */
#define FARADAY_CONST        96485          /* [Coulombs/mol] -  Faraday constant */
#define PH_CONST             2.303          /* Constant value used in Nernst formula for pH calculation */
#define AVOGADRO_NUMBER      8314            /* With [mV-Coulombs/˚K] - Avogadro number */
#define PH_ISO               7              /* Reference hydrogen ion concentration */
#define K_DEGREES            273.1          /* [˚K] - Kelvin degrees for 0˚C */
#define I_EXC                0.21                 /* Excitation current [uA] */


/**************************** Configuration parameters **********************/

#define  RMIN           1000                   /* Minimum value for RTD resistance */
#define  RMAX           1385                   /* Maximum value for RTD resistance */
#define  TMIN           0                      /* Minimum value for RTD temperature */
#define  TMAX           100                    /* Maximum value for RTD temperature */

#define  USE_IOUT2         NO          /* Select if you want to use output current from IOUT2 pin (YES) or you want to use default value 210 [uA] of the excitation current */
#define  TOLERANCE            0              /* Set a tolerance value for pH calculation */

#endif /* CN0326_H_ */
