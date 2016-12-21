/**
******************************************************************************
*   @file     ADT7420.h
*   @brief    Header file for ADT7420 Temperature IC.
*   @version  V0.1
*   @author   ADI
*   @date     December 2016
*  @par Revision History:
*  - V0.1, June 2016: initial version.
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
#include <stdio.h>
#include <math.h>

#include <ADuCM360.h>
#include <DioLib.h>

#include <ADT7420.h>
#include <Communication.h>
#include <Timer.h>


/****************************** Global Data ***********************************/

uint8_t ui8configAdt7420 = (FAULT_TRIGGER_4 | CT_PIN_POLARITY | INT_PIN_POLARITY | INT_CT_MODE |CONTINUOUS_CONVERSION_MODE | RESOLUTION_13_BITS);     /* Customize Configuration Register settings here using ADT7420.h file */

/************************* Global scope functions *****************************/

/**
   @brief Initialize the temperature sensor

   @return none

**/

void ADT7420_Init(void)
{
   uint16_t volatile ui16tempSetPoint = 0;

   /* Configure part using configuration parameters in ui8configAdt7420 */
   I2C_Write(ADT7420_ADDRESS, CONFIG_REG, ui8configAdt7420, 0x00 , I2C_WRITE_ONE_REG);

   /* Sets the High Temperature Monitoring Value */
   ui16tempSetPoint = ADT7420_Convert_Degrees_To_Hex(TEMP_HIGH_SETPOINT);
   I2C_Write(ADT7420_ADDRESS, T_HIGH_SETPOINT_MSB_REG, (ui16tempSetPoint>>8), (ui16tempSetPoint & 0x00FF), I2C_WRITE_TWO_REG);

   /* Sets the Low Temperature Monitoring Value */
   ui16tempSetPoint = ADT7420_Convert_Degrees_To_Hex(TEMP_LOW_SETPOINT);
   I2C_Write(ADT7420_ADDRESS, T_LOW_SETPOINT_MSB_REG, (ui16tempSetPoint>>8), (ui16tempSetPoint & 0x00FF), I2C_WRITE_TWO_REG);

   /* Sets the Critical Temperature Monitoring Value */
   ui16tempSetPoint = ADT7420_Convert_Degrees_To_Hex(TEMP_CRITICAL_SETPOINT);
   I2C_Write(ADT7420_ADDRESS, T_CRIT_SETPOINT_MSB_REG, (ui16tempSetPoint>>8), (ui16tempSetPoint & 0x00FF), I2C_WRITE_TWO_REG);

   /* Sets the Temperature Hysteresis for Monitoring */
   ui16tempSetPoint = ADT7420_Convert_Degrees_To_Hex(TEMP_HYSTERSIS_SETPOINT);
   I2C_Write(ADT7420_ADDRESS, T_HYST_SETPOINT_REG, (ui16tempSetPoint>>8), 0x00 , I2C_WRITE_ONE_REG);

}


/**
   @brief  Reads ADT7420 temperature data.

   @return ui16Data

**/

uint16_t ADT7420_Read_Temp(void)
{
   uint16_t ui16tempData;

   ui16tempData = I2C_Read(ADT7420_ADDRESS, TEMP_MSB_REG, I2C_READ_TWO_REG);      /*Read temp register */

   return ui16tempData;

}

/**
   @brief Read One ADT7420 register values.

   @param ui8regAddress - ADT7420 register value to read

   @return ui8regData

**/

uint16_t ADT7420_Read_One_Reg (uint8_t ui8regAddress)
{

   uint16_t ui16regData;

   ui16regData = I2C_Read(ADT7420_ADDRESS, ui8regAddress, I2C_READ_ONE_REG);

   return ui16regData;

}

/**
   @brief Read Two ADT7420 register values.

   @param ui8regAddress - ADT7420 register value to read

   @return ui16regData

**/

uint16_t ADT7420_Read_Two_Reg (uint8_t ui8regAddress)
{

   uint16_t ui16regData;

   ui16regData = I2C_Read(ADT7420_ADDRESS, ui8regAddress, I2C_READ_TWO_REG);

   return ui16regData;

}

/**
   @brief Write One ADT7420 register values.

   @param ui8regAddress - ADT7420 register value to write
   @param ui8Data - 8 bits of data

   @return none

**/

void ADT7420_Write_One_Reg (uint8_t ui8regAddress, uint8_t ui8Data)
{

   I2C_Write(ADT7420_ADDRESS, ui8regAddress, ui8Data, 0, I2C_WRITE_ONE_REG);

}

/**
   @brief Write Two ADT7420 register values.

   @param ui8regAddress - ADT7420 register value to write
   @param ui8Data - First 8 bits of data (msb)
   @param ui8Data2 - Second 8 bits of data (lsb)

   @return none

**/

void ADT7420_Write_Two_Reg (uint8_t ui8regAddress, uint8_t ui8Data, uint8_t ui8Data2)
{

   I2C_Write(ADT7420_ADDRESS, ui8regAddress, ui8Data, ui8Data2, I2C_WRITE_TWO_REG);

}

/**
   @brief Converts the degrees C to hex

   @param i16degrees - temperature in degrees C

   @return ui16degreeInHex - temperature digital word

**/

uint16_t ADT7420_Convert_Degrees_To_Hex (int16_t i16degrees)
{

   uint16_t ui16degreeInHex = 0;

   if((ui8configAdt7420 & 0x80) == 0x80){                        /* Checking to see if coding is 16- or 13- bit */
         if(i16degrees < 0){
               ui16degreeInHex = (i16degrees * 128) + 65536;            /* 16-bit negative temp to hex conversion */
            }
            else{
                  ui16degreeInHex = i16degrees * 128;            /* 16-bit positive temp to hex conversion */
            }
   }

   else{
         if(i16degrees < 0){
               ui16degreeInHex = (i16degrees * 16) + 8192;            /* 13-bit negative temp to hex conversion */
               ui16degreeInHex = ui16degreeInHex << 3;
         }

         else{
               ui16degreeInHex = i16degrees * 16;                /* 13-bit positive temp to hex conversion */
               ui16degreeInHex = ui16degreeInHex << 3;
         }
   }

   return ui16degreeInHex;
}


/**
   @brief Converts hex to degrees C

   @param ui16tempResults - temperature data in hex

   @return fTemp - temperature in degrees C

**/

float ADT7420_Convert_Hex_To_Degrees (ui16tempResults){

   float fTemp = 0.0;

   //Check the status of the temperature sign bit (MSB)
     if(( ui16tempResults & 0x8000 ) == 0x8000) {               /* If sign bit is 1 use the negative temperature equation */

           if((ui8configAdt7420 & 0x80) == 0x80){
                 fTemp = (((float)ui16tempResults - 65536)/128);         /* 16-bit temperature word data */
           }

           else{
                 ui16tempResults = ui16tempResults >> 3;
                 fTemp = (((float)ui16tempResults - 8192)/16);          /* 13-bit temperature word data */
           }
     }

     else{                                                     /*If sign bit is 0, use the positive temperature equation */

           if((ui8configAdt7420 & 0x80) == 0x80){
                  fTemp = (float)ui16tempResults/128;                   /* 16-bit temperature word data */
           }

           else{
                 ui16tempResults = ui16tempResults >> 3;
                 fTemp = ((float)ui16tempResults/16);                   /* 13-bit temperature word data */
           }

     }
     return fTemp;
}

/**
   @brief Powerdown ADT7420.

   @return none

**/

void ADT7420_Power_Down ()
{

   I2C_Write(ADT7420_ADDRESS, CONFIG_REG, (ui8configAdt7420 | 0x60), 0x00 , I2C_WRITE_ONE_REG);

}


/**
   @brief Powerup ADT7420.

   @return none

**/

void ADT7420_Power_Up ()
{

   I2C_Write(ADT7420_ADDRESS, CONFIG_REG, (ui8configAdt7420 & 0x9F), 0x00 , I2C_WRITE_ONE_REG);
   //timer_sleep(100);                     /* Wait 1 ms to wake up  */

}
