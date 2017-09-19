/**
******************************************************************************
*   @file     main.c
*   @brief    Project main source file for CN0357
*   @version  V0.1
*   @author   ADI
*   @date     December 2015
*   @par Revision History:
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


/***************************** Include Files **********************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <DioLib.h>
#include <ADuCM360.h>

#include "ADXL362.h"
#include "Lcd.h"
#include "Communication.h"
#include "Timer.h"
#include "ISM43340.h"
#include "Services.h"
#include "User_Settings.h"

#include "UrtLib.h"

/********************************* Definitions ********************************/

/* Sample pragmas to cope with warnings. Please note the related line at
  the end of this function, used to pop the compiler diagnostics status. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/********************************** Macros ****************************************************************************/
#define BUFFER_SIZE 3000

/******************************************Variables*******************************************************************/
char ISM43340_ResponseBuffer[BUFFER_SIZE];
char flag = 0;
int count = 0;

/************************************* Main ****************************************************************************/
/**
   @brief The main application function

   @return the function contains infinite loop and never returns/

**/
/**********************************************************************************************************************/
int main(int argc, char *argv[])
{
   // Main Variables
   char status = -1, send_location = 1, init_system = 1, send_appinfo = 1;
   connection state = STATE_WIFI_CONNECT;
   error_log error __attribute__((unused)) = NONE;


   // Initialize components of the system
   timer_start();
   ADXL362_Init();
   ISM43340_Init();

   while (1)
      {
          switch(state)
          {
             case STATE_WIFI_CONNECT:
                if(init_system == ENABLE)
                {
                      Disconnect_WiFi();                                                     // During system initialization, disconnect from local Wi-Fi network
                      init_system = DISABLE;
                 }
                 status = Check_WifiConnection(ISM43340_ResponseBuffer);                     // Check connection to local Wi-Fi network
                 if(status == WIFI_CONNECTED)
                 {
                       state = STATE_SERVER_CONNECT;
                 }
                 else
                 {
                       Connect_WiFi(ssid,password,security_type);                            // Connect to local Wi-Fi network
                       status = Check_WifiConnection(ISM43340_ResponseBuffer);               // Check connection to local Wi-Fi network
                       if(status == WIFI_CONNECTED)
                       {
                             state = STATE_SERVER_CONNECT;
                       }
                      else
                      {
                            error = WIFI_CONNECT_ERROR;
                            state = STATE_WIFI_CONNECT;
                      }
                  }
                  status = -1;                                                               // Clear buffer used for storing responses from the Wi-Fi module
                  memset(ISM43340_ResponseBuffer,0,BUFFER_SIZE);
                  count = 0;
              break;


              case STATE_SERVER_CONNECT:
                 status = Connect_Server(instance_url,ISM43340_ResponseBuffer);              // Connect to the Analog Devices Connect instance
                  if(status == SERVER_CONNECTED)
                  {
                       state = STATE_CHECK_THING_EXISTS;
                  }
                  else
                  {
                      error = SERVER_CONNECTION_ERROR;
                      state = STATE_WIFI_CONNECT;
                  }
                  status = -1;
                  memset(ISM43340_ResponseBuffer,0,BUFFER_SIZE);                            // Clear buffer used for storing responses from the Wi-Fi module
                  count = 0;

              break;


              case STATE_CHECK_THING_EXISTS:
                 status = Check_ThingExists(ISM43340_ResponseBuffer);                       // Check if Thing with Thing name set in the User_Settings.h file already exists in the Analog Devices Connect instance.
                 if(status == THING_EXISTS)
                 {
                       error = THING_ALREADY_EXISTS;
                       state = STATE_SEND_LOCATION;
                 }
                 else
                 {
                       state = STATE_CREATE_THING;
                 }
                 status = -1;
                 memset(ISM43340_ResponseBuffer,0,BUFFER_SIZE);                             // Clear buffer used for storing responses from the Wi-Fi module
                 count = 0;
              break;


              case STATE_CREATE_THING :
                 status = Create_Thing(ISM43340_ResponseBuffer);                            // Create a Thing in the Analog Devices Connect instance
                 if(status == THING_CREATED)
                 {
                       state = STATE_SEND_LOCATION;
                 }
                 else
                 {
                       error = THING_CREATION_ERROR;
                       state = STATE_WIFI_CONNECT;
                 }
                 status = -1;
                 memset(ISM43340_ResponseBuffer,0,BUFFER_SIZE);                            // Clear buffer used for storing responses from the Wi-Fi module
                 count = 0;

              break;


              case STATE_SEND_LOCATION:
                  if(send_location == ENABLE)
                  {
                     status = Send_Location(ISM43340_ResponseBuffer, latitude, longitude, elevation);    // Send device location to the Analog Devices Connect instance
                     if(status == LOCATION_SENT)
                     {
                         send_location = DISABLE;
                         state = STATE_SEND_APPINFO;
                     }
                     else
                     {
                        error = SEND_LOCATION_ERROR;
                        state = STATE_WIFI_CONNECT;
                     }
                 }
                 else
                 {
                     state = STATE_SEND_APPINFO;
                 }
                 status = -1;
                 memset(ISM43340_ResponseBuffer,0,BUFFER_SIZE);                           // Clear buffer used for storing responses from the Wi-Fi module
                 count = 0;
              break;


              case STATE_SEND_APPINFO:
                 if(send_appinfo == ENABLE)
                 {
                       status = Send_AppInfo(ISM43340_ResponseBuffer);                   // Send information of the application/firmware running on the ISM43340 connectivity module to the Analog devices Connect instance.
                       if(status == APPINFO_SENT)
                       {
                             send_appinfo = DISABLE;
                             state = STATE_GET_SENSORDATA;
                       }
                       else
                       {
                             error = SEND_APPINFO_ERROR;
                             state = STATE_WIFI_CONNECT;
                       }
                  }
                  else
                  {
                       state = STATE_GET_SENSORDATA;
                  }
                  status = -1;
                  memset(ISM43340_ResponseBuffer,0,BUFFER_SIZE);                         // Clear buffer used for storing responses from the Wi-Fi module
                  count = 0;
              break;


              case STATE_GET_SENSORDATA:
                 Lcd_Run();
                 state = STATE_SEND_SENSORDATA;                                         // Read accelerometer data and display it on the LCD
              break;


              case STATE_SEND_SENSORDATA:
                 status = Send_SensorData(i16SensorX, i16SensorY, i16SensorZ, ISM43340_ResponseBuffer);  // Send sensor data to the Analog Devices Connect instance
                 if(status == DATA_SENT)
                 {
                       state = STATE_GET_SENSORDATA;
                 }
                 else
                 {
                      error = SEND_SENSORDATA_ERROR;
                      state = STATE_WIFI_CONNECT;
                 }
                 status = -1;
                 memset(ISM43340_ResponseBuffer,0,BUFFER_SIZE);                       // Clear buffer used for storing responses from the Wi-Fi module
                 count = 0;

                 break;

                 default:
                 break;
              }
  }

}


/* UART Interrupt Handler */
void UART_Int_Handler (void)
{

    unsigned short  status;
    char c;
    status = UrtIntSta(pADI_UART);                                                 // Check UART status
    if (status & COMIIR_NINT)                                                      // Check if UART is busy
    {
        return;
    }
   switch (status & COMIIR_STA_MSK) {                                              // Check what command to execute

   case COMIIR_STA_RXBUFFULL:                                                      // Check if UART register is available to be read

        UART_ReadChar(&c);                                                         // Read character from UART
        if(count == (BUFFER_SIZE-1))
        {
            count = 0;
        }
        ISM43340_ResponseBuffer[count] = c;
        count++;

        // Set flag if eSWifi Module has returned to the AT Command Mode (indicated by reception of '>' character in eSWifi module's response
        if( c == 62)
        {
            flag++;
        }

    break;

    case COMIIR_STA_TXBUFEMPTY:                                                     // Check if UART register is available to be written
        if (uart_tcnt)                                                              // Check UART counter
           {
            uart_tbusy = UART_TRUE;                                                 // UART is busy with writing
            uart_tcnt--;                                                            // Decrement  UART counter
            UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);                // Write character to UART
            if (uart_tpos == UART_TX_BUFFER_SIZE)                                   // Check if TX buffer is full
            {
                uart_tpos = 0;                                                      // Reset buffer counter
            }

        }
        else
        {
            uart_tbusy = UART_FALSE;                                                // UART is no longer busy with writing
        }

    break;

    default:
      ;
    }

}

#pragma GCC diagnostic pop

/* End Of File */
