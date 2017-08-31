/**
******************************************************************************
*   @file     AD5270.h
*   @brief    Header file for AD5270 Rheostat.
*   @version  V0.1
*   @author   ADI
*   @date     December 2015
*   @par Revision History:
*  - V0.1, December 2015: initial version.
*
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
#ifndef AD5270_H_
#define AD5270_H_

/*****************************************************************************/
/********************************  Definitions *******************************/
/*****************************************************************************/

//Commands
#define NO_OP                 0x00               /* Write to the RDAC Register */
#define WRITE_RDAC            0x04               /* Write to the RDAC Register */
#define READ_RDAC             0x08               /* Read from the RDAC Register */
#define STORE_50TP            0x0C               /* Write to the RDAC to memory */
#define SW_RST                0x10               /* Software reset to last memory location */
#define READ_50TP_CONTENTS    0x14               /* Read the last memory contents */
#define READ_50TP_ADDRESS     0x18               /* Read the last memory address */
#define WRITE_CTRL_REG        0x1C               /* Write to the control Register */
#define READ_CTRL_REG         0x20               /* Read from the control Register */
#define SW_SHUTDOWN           0x24               /* Software shutdown (0) - Normal, (1) - Shutdown */

#define HI_Zupper             0x80           /* Get the SDO line ready for High Z */
#define HI_Zlower             0x01           /* Puts AD5270 into High Z mode */

/*************************** Functions prototypes *****************************/

void AD5270_Init(float fResistorValue);
uint16_t AD5270_CalcRDAC(float fresistor);
void AD5270_WriteReg(uint8_t ui8command, uint16_t ui16value);
void AD5270_SetSDOHiZ(void);


#endif /* AD5270_H_ */

/* End Of File */
