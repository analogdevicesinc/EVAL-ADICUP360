/*!
 *****************************************************************************
 * @file:    AD5270.h
 * @brief:   AD5270 Rheostat
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
