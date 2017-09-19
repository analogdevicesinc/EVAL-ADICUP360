/**
******************************************************************************
*   @file     eSWifi.h
*   @brief    Header file for eS-Wifi Module
*   @version  V0.1
*   @author   ADI
*   @date     May 2016
*   @par Revision History:
*  - V0.1, December 2015: initial version.
*
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
*****************************************************************************************
**/

#ifndef INCLUDE_ESWIFI_H_
#define INCLUDE_ESWIFI_H_

/******************************* Macro Definitions***************************************/
#define ENABLE 1
#define DISABLE 0
#define WIFI_CONNECTED 1
#define WIFI_DISCONNECTED 0
#define SERVER_CONNECTED 2
#define PROPERTY_ADDED 1
#define APPINFO_SENT 1
#define LOCATION_SENT 1
#define DATA_SENT 1
#define THING_EXISTS 1
#define THING_CREATED 1

/******************************* Variable Definitions ************************************/

typedef enum
{
   STATE_WIFI_CONNECT, STATE_SERVER_CONNECT, STATE_CHECK_THING, STATE_CREATE_THING,
   STATE_CHECK_THING_EXISTS, STATE_SEND_LOCATION, STATE_SEND_APPINFO, STATE_GET_SENSORDATA, STATE_SEND_SENSORDATA
} connection;



typedef enum
{
   NONE, WIFI_CONNECT_ERROR, SERVER_CONNECTION_ERROR, THING_ALREADY_EXISTS,
   THING_CREATION_ERROR, ADD_PROPERTY_DEFINITION_ERROR, SEND_LOCATION_ERROR,
   SEND_APPINFO_ERROR,SEND_SENSORDATA_ERROR
} error_log;

/*************************** Functions Prototypes *****************************/

void Return_ATCommandMode();
void ISM43340_Init(void);
void Disconnect_WiFi();
void Connect_WiFi(char *wifi_ssid, char *wifi_password, char *wifi_sectype);
char Connect_Server(char *url, char* data_buffer);
void Write_Transport_Data(char *data);
char Check_WifiConnection(char* data_buffer);
char Check_ServerResponse(char* data_buffer);
void Get_AppInfo();
/******************************************************************************/
#endif /* INCLUDE_ESWIFI_H_ */

