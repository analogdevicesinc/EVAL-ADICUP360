/**
******************************************************************************
*   @file     eSWifi.c
*   @brief    Source file for eS-Wifi Module.
*   @version  V0.1
*   @author   ADI
*   @date     May 2016
*   @par Revision History:
*  - V0.1, May 2016: initial version.
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
**/

/***************************** Include Files **********************************/
#include <HTTP_REST_API.h>
#include <ISM43340.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "Communication.h"
#include "Timer.h"


#include "ADuCM360.h"

#include "SpiLib.h"
#include "DioLib.h"
#include "UrtLib.h"
#include "ClkLib.h"

/************************** Variable Definitions ***************************************************************************************/
//#define SSL
extern char flag;
/************************** Function Definitions ***************************************************************************************/

/**
   @brief Initialize ISM43340 WiFi connectivity module

   @param none

   @return none

**/
void ISM43340_Init()
{
   // Initialize UART
   UART_Init (B115200, COMLCR_WLS_8BITS);
}


/**
   @brief Checks if ISM43340 module is in AT Command Mode waiting for next AT Command

   @param none

   @return none

**/

void Return_ATCommandMode()
{
   while(flag != 1);
   flag = 0;
}


/**
   @brief Connects ISM43340 module to Local WiFi Network

   @param wifi_ssid - Local WiFi Network SSID
   @param wifi_password - Local WiFi Network Password
   @param wifi_sectype - Local WiFi Network Security Type

   @return none

**/

void Connect_WiFi(char *wifi_ssid, char *wifi_password, char *wifi_sectype)
{
   // Setup GPIO3 to control Red LED
   AppPrintf("G4=3,1\r");
   Return_ATCommandMode();

   // Setup GPIO4 to control Green LED
   AppPrintf("G4=4,1\r");
   Return_ATCommandMode();

   // Set Red LED to OFF
   AppPrintf("G3=0,1,0\r");
   Return_ATCommandMode();

   // Set Green LED - OFF
   AppPrintf("G3=1,1,0\r");
   Return_ATCommandMode();

   // Set Red and Green LEDs for link status and link activity respectively
   AppPrintf("WL=4,3,1\r");
   Return_ATCommandMode();

   // No Usage message on error
   AppPrintf("MT=1\r");
   Return_ATCommandMode();

   // TCP API message timeout
   AppPrintf("PY=10000\r");
   Return_ATCommandMode();

   // Set Local WiFi Network SSID
   AppPrintf("C1=%s\r",wifi_ssid);
   Return_ATCommandMode();

   // Set Local WiFi Network Password
   AppPrintf("C2=%s\r",wifi_password);
   Return_ATCommandMode();

   // Set Local WiFi Network Security Type(WPA)
   AppPrintf("C3=%s\r",wifi_sectype);
   Return_ATCommandMode();

   // Set Network DHCP
   AppPrintf("C4=1\r");
   Return_ATCommandMode();

   // Set Network Join Retry Count
   AppPrintf("CB=10\r");
   Return_ATCommandMode();

   // Join Local WiFi Network
   AppPrintf("C0\r");
   Return_ATCommandMode();
}


/**
   @brief Connects ISM43340 Module (in Client Mode) to Analog Devices Connect Server

   @param url - Cloud Instance URL
   @param data_buffer - ISM43340 module Response Buffer

   @return ServerConnection_status - Status of connection of ISM43340 module to Cloud Server

**/

char Connect_Server(char *url, char* data_buffer)
{
   char ServerConnection_status = -1;

   // Suppress DHCP assigned msg from being sent to host
   AppPrintf("MS=1\r");
   Return_ATCommandMode();

   // Stop Transport Client
   AppPrintf("P6=0\r");
   Return_ATCommandMode();

   // Set/Display Communication socket (0-3)
   AppPrintf("P0=0\r");
   Return_ATCommandMode();

   // Set Transport Protocol (0=TCP, 1=UDP, 2=UDP Lite, 3=TCP-SSL)
   #ifdef SSL
      AppPrintf("P1=3\r");
      Return_ATCommandMode();

      // Set Transport Remote Port Number (80=HTTP, 443=HTTPS)
      AppPrintf("P4=443\r");
      Return_ATCommandMode();

      // Set SSL Certificate Authentication (0 = No Cert Validation, 1 = Required)
      AppPrintf("P9=0\r");
      Return_ATCommandMode();
   #endif

   #ifndef SSL
      // Set Transport Protocol (0=TCP, 1=UDP, 2=UDP Lite, 3=TCP-SSL)
      AppPrintf("P1=0\r");
      Return_ATCommandMode();

      // Set Transport Remote Port Number (80=HTTP, 443=HTTPS)
      AppPrintf("P4=80\r");
      Return_ATCommandMode();;
   #endif

   // DNS Lookup
   AppPrintf("D0=%s\r",url);
   Return_ATCommandMode();

   // Enable & Set TCP Keep-Alive Time-to-idle
   AppPrintf("PK=1,3000\r");
   Return_ATCommandMode();

   // Start Transport Client
   AppPrintf("P6=1\r");
   Return_ATCommandMode();

   // Set Read Transport Packet size
   AppPrintf("R1=1200\r");
   Return_ATCommandMode();

   // Set Read Transport timeout(0-30000ms)
   AppPrintf("R2=500\r");
   Return_ATCommandMode();

   // Check if successfully connected to Server
   ServerConnection_status = Check_ServerResponse(data_buffer);

   return(ServerConnection_status);
}


/**
   @brief Writes Transport Data to Analog devices Connect server

   @param data - transport data

   @return none

**/

void Write_Transport_Data(char *data)
{
   int i=0;

   // Set Write Transport Timeout
   AppPrintf("S2=250\r");
   Return_ATCommandMode();

   // Set Write Transport Packet size
   AppPrintf("S1=%d\r",strlen(data));
   Return_ATCommandMode();

   // Write Transport Data
   AppPrintf("S0\r%s",data);
   Return_ATCommandMode();

   for(i=0;i<8;i++)
   {
         // Read Transport Data
         AppPrintf("R0\r");
         Return_ATCommandMode();
   }
   timer_sleep(50);
}


/**
   @brief Disconnects ISM43340 module from Local WiFi Network

   @param none

   @return none

**/
void Disconnect_WiFi()
{
   // Disconnect from Local WiFi Network
   AppPrintf("CD\r");
   Return_ATCommandMode();
}


/**
   @brief Gets Information of firmware/application running on the Wi-Fi Module

   @param data_buffer - ISM43340 module Response Buffer

   @return none

**/
void Get_AppInfo()
{
      AppPrintf("I?\r");
      Return_ATCommandMode();
}


/**
   @brief Checks ISM43340 module's connection to the Local WiFi Network

   @param data_buffer - ISM43340 module Response Buffer

   @return WiFiConnection_status - Status of connection of ISM43340 module to Local WiFi Network

**/

char Check_WifiConnection(char* data_buffer)
{
   char connection_success[] = "1";
   char WifiConnection_status = -1;
   char *ret;

   // Get Connection status (1=Connected, 0=Disconnected)
   AppPrintf("CS\r");
   Return_ATCommandMode();

   ret = strstr(data_buffer,connection_success);
   if(ret==NULL)
   {
       // Unsuccessful in connecting to local WiFi Network
       WifiConnection_status = 0;
   }
   else
   {
      // Successful in Connecting to local WiFi Network
      WifiConnection_status = 1;
   }
   return(WifiConnection_status);
}


/**
   @brief Checks responses from the Analog Devices Connect server

   @param data_buffer - ISM43340 module Response Buffer

   @return Request_status - Cloud Server Response status

**/

char Check_ServerResponse(char *data_buffer)
{
   char response_200OK[10],response_error[10];
   char *ret_200OK, *ret_error;
   char Request_status;

   sprintf(response_200OK, "200 OK");
   sprintf(response_error, "ERROR");

   ret_200OK = strstr(data_buffer,response_200OK);
   ret_error = strstr(data_buffer,response_error);

   if(ret_200OK != NULL)
   {
          // HTTP Request success
          Request_status = 1;
   }
   else if(ret_error == NULL)
   {
         // Successfully connected to server
         Request_status = 2;
   }
   else
   {
         // Request failure
         Request_status = 0;
   }


   return(Request_status);

}


/****************************************************************************************************************************************/



