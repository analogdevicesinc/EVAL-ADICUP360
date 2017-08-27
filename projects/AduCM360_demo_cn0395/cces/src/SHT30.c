/******************************************************************************
*   @file     SHT30.c
*   @brief    Source file for handling SHT30 sensor
*   @version  V0.1
*   @author   ADI
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
*******************************************************************************/

/***************************** Include Files **********************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include <DioLib.h>
#include "Communication.h"
#include "SHT30.h"
#include "Timer.h"
#include "I2cLib.h"

/**************************** Global functions *******************************/

void SHT30_Update(float *fTemp,  float *fHum)
{
   uint16_t ui16RawTemp;
   uint16_t ui16RawHum;

   I2C_Write(SHT30_MEAS_HIGHREP_PERIODIC);

   timer_sleep(5); // delay 5ms

   I2C_Read();

   while (i2c_rx_cnt < 6); // wait until 6 bytes are received

   i2c_rx_cnt = 0; // reload the i2c rx buffer index

   ui16RawTemp = ((uint16_t)i2c_rx_buf[0] << 8) | i2c_rx_buf[1];
   ui16RawHum = ((uint16_t)i2c_rx_buf[3] << 8) | i2c_rx_buf[4];

   *fTemp = (float)(ui16RawTemp * 175) / (pow(2, 16) - 1) - 45;
   *fHum = (float)(ui16RawHum * 100) / (pow(2, 16) - 1);
}
