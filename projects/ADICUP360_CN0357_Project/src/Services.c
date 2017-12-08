/*!
 *****************************************************************************
 * @file:    Services.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2016-2017 Analog Devices, Inc.

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

/***************************** Include Files **********************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "Timer.h"
#include "Communication.h"
#include "Services.h"
#include "ISM43340.h"
#include "HTTP_REST_API.h"

/************************************************* Function Definitions *****************************************************************/

/**
   @brief Sends sensor data the Analog Devices Connect instance

   @param data -  CO Gas Concentration(ppm)
   @param data_buffer - ISM43340 Response buffer

   @return SendData_status - Send Sensor Data HTTP REST API Call status

**/


char Send_SensorData(float data, char *data_buffer)
{
   char *http_request;
   char SendData_status = -1;

   // Construct HTTP REST API call for sending sensor data to cloud
   http_request = SendData_Request(data);

   // Write Transport Data
   Write_Transport_Data(http_request);

   // Check for broken TCP Connections
   SendData_status = Check_ServerResponse(data_buffer);

   return(SendData_status);
}



/**
   @brief Creates a Thing to represent the device in the Analog Devices Connect instance

   @param data_buffer - ISM43340 Response buffer

   @return ThingCreate_status - Create Thing HTTP REST API Call status

**/

char Create_Thing(char* data_buffer)
{
   char ThingCreate_status = -1;
   char *http_request;

   // HTTP REST API Call to Create Thing
   http_request = CreateThing_Request();
   Write_Transport_Data(http_request);

   // Check status HTTP REST API Call
   ThingCreate_status = Check_ServerResponse(data_buffer);

   if(ThingCreate_status == SUCCESS)
   {
         // HTTP REST API Call to Enable Thing
         http_request = EnableThing_Request();
         Write_Transport_Data(http_request);

         // Check status HTTP REST API Call
         ThingCreate_status = Check_ServerResponse(data_buffer);

         if(ThingCreate_status == SUCCESS)
         {
               // HTTP REST API Call to Restart Thing
               http_request = RestartThing_Request();
               Write_Transport_Data(http_request);

               // Check status HTTP REST API Call
               ThingCreate_status = Check_ServerResponse(data_buffer);

         }
   }
   return (ThingCreate_status);
}


/**
   @brief Check if a Thing already exists in the Analog Devices Connect instance

   @param data_buffer -  ISM43340 Response buffer

   @return ThingExists_status - Check Thing Exists HTTP REST API Call status

**/

char Check_ThingExists(char *data_buffer)
{
    char ThingExists_status = -1;
    char *http_request;

    // HTTP REST API Call to Check if Thing Exists  */
    http_request = GetPropertyValues_Request();
    Write_Transport_Data(http_request);

    ThingExists_status = Check_ServerResponse(data_buffer);
    return(ThingExists_status);

}


/**
   @brief Sends Information of the Application running on the Wi-Fi module to cloud

   @param data_buffer -  ISM43340 Response buffer

   @return AppInfoSet_status - Send Application Information HTTP REST API Call status

**/
char Send_AppInfo(char *data_buffer)
{
   char AppInfoSent_status = -1;
   char *http_request;

   int i=2,j=0;
   char product_id[30] ={0}, fw_rev[30]={0}, api_rev[30]={0}, stack_rev[30]={0}, rtos_rev[30]={0}, cpu_clk[30]={0};

   Get_AppInfo();

        // Identifying the components of the application information obtained from the Wi-Fi module
        while(data_buffer[i] != 44)
        {
              product_id[j] = data_buffer[i];
              i++;j++;
        }
        i++;j=0;
        while(data_buffer[i] != 44)
        {
              fw_rev[j] = data_buffer[i];
              i++;j++;
        }
        i++;j=0;
        while(data_buffer[i] != 44)
        {
               api_rev[j] = data_buffer[i];
               i++;j++;
        }
        i++;j=0;
        while(data_buffer[i] != 44)
        {
               stack_rev[j] = data_buffer[i];
               i++;j++;
        }
        i++;j=0;
        while(data_buffer[i] != 44)
        {
               rtos_rev[j] = data_buffer[i];
               i++;j++;
        }
        i++;j=0;
        while(data_buffer[i] != 44)
        {
                cpu_clk[j] = data_buffer[i];
                i++;j++;
        }

   // Build Send Application Information Request
   http_request = SendAppInfo_Request(product_id, fw_rev, api_rev, stack_rev, rtos_rev, cpu_clk);

   // Write Transport Data
   Write_Transport_Data(http_request);

   // Check response from server
   AppInfoSent_status = Check_ServerResponse(data_buffer);

    return(AppInfoSent_status);
}


/**
   @brief Sends Device Location Information to cloud

   @param data_buffer -  ISM43340 Response buffer
   @param latitude    - Latitude of device location
   @param longitude   - Longitude of device location
   @param elevation   - Elevation of device location

   @return LocationSent_status - Send Location HTTP REST API Call status

**/
char Send_Location(char *data_buffer, float latitude, float longitude, float elevation)
{
   char LocationSent_status = -1;
   char *http_request;

    // Build Send Location Request
    http_request = SendLocation_Request(latitude, longitude, elevation);

    // Write Transport Data
    Write_Transport_Data(http_request);

   // Check response from server
   LocationSent_status = Check_ServerResponse(data_buffer);

   return(LocationSent_status);
}
/********************************************************************************************************************/
