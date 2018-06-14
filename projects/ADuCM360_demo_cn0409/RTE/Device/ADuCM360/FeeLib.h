/*******************************************************************************
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
*******************************************************************************/

/**
 *****************************************************************************
   @file     FeeLib.h
   @brief    Set of Flash peripheral functions.
   @version  V0.4
   @author   ADI
   @date     October 2015
   @par Revision History:
   - V0.1, October 2012: initial version.
   - V0.2, November 2012: Added warnings about 64k parts
   - V0.3, October 2015: Coding style cleanup - no functional changes.
   - V0.4, October 2015: Use Standard Integer Types, prefer unsigned types, add include and C++ guards.

**/
#ifndef __ADUCM36X_FEELIB_H
#define __ADUCM36X_FEELIB_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <ADuCM360.h>

#define FA_KEYH (*(volatile uint32_t *) 0x1FFF4)
#define FA_KEYL (*(volatile uint32_t *) 0x1FFF0)
#define WR_PR (*(volatile uint32_t *) 0x1FFF8)


extern uint32_t FeeMErs(void);
extern uint32_t FeePErs(uint32_t lPage);
extern uint32_t FeeWrPro(uint32_t lKey);
extern uint32_t FeeWrProTmp(uint32_t lKey);
extern uint32_t FeeRdProTmp(uint32_t iMde);
extern uint32_t FeeWrEn(uint32_t iMde);
extern uint32_t FeeSta(void);
extern uint32_t FeeFAKey(uint64_t udKey);
extern uint32_t FeeIntAbt(uint32_t iAEN0, uint32_t iAEN1, uint32_t iAEN2);
extern uint32_t FeeAbtAdr(void);
extern uint32_t FeeSign(uint32_t ulStartAddr, uint32_t ulEndAddr);
extern uint32_t FeeSig(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADUCM36X_FEELIB_H */
