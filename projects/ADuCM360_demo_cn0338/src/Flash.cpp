/**
******************************************************************************
*   @file     Flash.cpp
*   @brief    Source file for flash memory management.
*   @version  V0.1
*   @author   ADI
*   @date     February 2016
*  @par Revision History:
*  - V0.1, November 2016: initial version.
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
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
*/

/***************************** Include Files **********************************/
#include <stdio.h>
#include <cassert>
#include <cmath>

#include <ADuCM360.h>
#include <FeeLib.h>
#include <Flash.h>


using namespace std;

/****************************** Global variables and types *****************************/
union UOptions {
   volatile unsigned long m_padding[512 / sizeof(unsigned long)];
   SOptions m_options;
};

const UOptions Options __attribute__((aligned (512))) = {};

UOptions *const pOptions = (UOptions *) (&Options);

SOptions::SOptions(void)
{
   baud_rate = 115200;
   ndir_frequency = 0.25f;
   adc_frequency = 10.0008f;

   rising_ms = 500;
   falling_ms = 500;

   zero = 1.2f;
   span = 1.f;
   b = 4.5e-2f;
   c = 1.f;
   k = 298.15f;
   serial_num = 0;

   init = false;
}

/****************************** Global functions *****************************/

/**
   @brief F

   @return none
**/
void Flash_Init(void)
{
   pADI_FEE->FEEADR0H = ((((unsigned long) (&Options)) >> 16) & 0x1);
   pADI_FEE->FEEADR0L = (((unsigned long) (&Options)) & 0xfe00);

   pADI_FEE->FEEADR1H = ((((unsigned long) (&Options)) >> 16) & 0x1); // for CRC verify
   pADI_FEE->FEEADR1L = (((unsigned long) (&Options)) & 0xfe00);
}

/****************************** Global functions *****************************/

/**
   @brief Set options values

   @param options - parameters to be set

   @return none
**/
void Flash_SetOptions(const SOptions options)
{
   UOptions OptionsToSave = {};
   OptionsToSave.m_options = options;

   SOptions default_options;

   if (Options.m_options.serial_num != default_options.serial_num) { // only can change once
      OptionsToSave.m_options.serial_num = Options.m_options.serial_num;
   }

   pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY1;
   pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY2;

   pADI_FEE->FEECMD = FEECMD_CMD_ERASEPAGE;

   while (FEESTA_CMDBUSY_BBA)
      ;

   FEECON0_WREN_BBA = true;

   for (unsigned int a = 0; a < ((512 / sizeof(unsigned long)) - 1); ++a) { // most significant 4 bytes for CRC
      pOptions->m_padding[a] = OptionsToSave.m_padding[a];

      while (FEESTA_WRBUSY_BBA)
         ;
   }

   pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY1;
   pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY2;

   pADI_FEE->FEECMD = FEECMD_CMD_SIGN;

   while (FEESTA_CMDBUSY_BBA)
      ;

   auto sig = (((pADI_FEE->FEESIGH & 0xff)) << 16) | (pADI_FEE->FEESIGL & 0xffff);

   pOptions->m_padding[(512 / sizeof(unsigned long)) - 1] = sig;

   while (FEESTA_WRBUSY_BBA)
      ;

   FEECON0_WREN_BBA = false;
}

/**
   @brief Get options values

   @return options - parameters to be read
**/
SOptions Flash_GetOptions(void)
{
   return Options.m_options;
}


/**
   @brief Verify options values

   @return bool - valid or not
**/
bool Flash_VerifyOptions(void)
{

   assert(sizeof(SOptions) <= (512 - sizeof(unsigned long)));

   pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY1;
   pADI_FEE->FEEKEY = FEEKEY_VALUE_USERKEY2;

   pADI_FEE->FEECMD = FEECMD_CMD_SIGN;

   while (FEESTA_CMDBUSY_BBA)
      ;

   return (Options.m_padding[(512 / sizeof(unsigned long)) - 1]
           == FeeSig());
}

