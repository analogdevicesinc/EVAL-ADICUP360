/**
*******************************************************************************************************************
*   @file     User_Settings.h
*   @brief    Header file for Local WiFi Network and Cloud Instance Settings.
*   @version  V0.1
*   @author   ADI
*   @date     May 2065
*   @par Revision History:
*  - V0.1, December 2015: initial version.
*
*
********************************************************************************************************************
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
***********************************************************************************************************************
**/

#ifndef USER_SETTINGS_H_
#define USER_SETTINGS_H_


/********************************** Enter information ****************************************************************/

/* Enter Local WiFi Network Information */
char ssid[] = "";                                                // Enter SSID of WiFi Network
char password[] = "";                                            // Enter Password of WiFi Network
char security_type[] = "";                                       // Enter security type
                                                                 // (0=Open, 1=WEP, 2=WPA, 3=WPA2, 4=WPA+WPA2, 5=WPA2 TKIP)
/* Enter Cloud Instance Information */
char instance_url[] = "";                                        // Enter Analog Devices Connect Instance URL
char app_key[] = "";                                             // Enter Application Key

/* Enter Thing Name */
char Thing_Name[] = "ADICUP360_CN0397_Thing";                    // Enter Thing Name
char Thing_Description[] = "Thing Created by Device";            // Enter Thing Description

/* Enter Location of Device here*/                               // Enter Device location
float latitude = 42.2105;
float longitude = -71.1806;
float elevation = 0;

/***********************************************************************************************************************/

char ThingTemplate_Name[] = "ADICUP360_CN0397_ThingTemplate";
char Property1_Name[] = "RedLight_Intensity";
char Property2_Name[] = "GreenLight_Intensity";
char Property3_Name[] = "BlueLight_Intensity";

#endif /* USER_SETTINGS_H_ */

/*********************************************************************************************************************************************************************/
