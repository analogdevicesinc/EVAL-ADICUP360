/**
******************************************************************************
*   @file     CN0326.c
*   @brief    Source file for CN0326 application
*   @version  V0.1
*   @author   ADI
*   @date     December 2015
*  @par Revision History:
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
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include <DioLib.h>
#include <UrtLib.h>

#include "AD7793.h"
#include "CN0326.h"
#include "Communication.h"
#include "Timer.h"



/********************************* Definitions ********************************/
#if(USE_IOUT2 == YES)
float iout2_calibration;   /* [mA]  */
#endif

/************************* Variable Definitions ******************************/

/* Available commands */
char *CmdCommands[] = {
   "help",
   "calibrate",
   "temp",
   "ph",
   "reset",
   ""
};

/* Functions for available commands */
cmdFunc CmdFun[] = {
   CN0326_CmdHelp,
   CN0326_CmdCalibration,
   CN0326_CmdTemp,
   CN0326_CmdPH,
   CN0326_CmdReset,
   NULL
};

/****************************** Global functions *****************************/

/**
   @brief Initialization part

   @return none

**/
void CN0326_Init(void)
{
#if(USE_IOUT2 == YES)
   uint32_t ui32result;
   int32_t i32voltage;
#endif

   UART_Init(B9600, COMLCR_WLS_8BITS);  /* UART initialization */

   AD7793_Calibrate(AD7793_CH_AIN1P_AIN1M, CAL_INT_ZERO_MODE);
   AD7793_Calibrate(AD7793_CH_AIN1P_AIN1M, CAL_INT_FULL_MODE);
   AD7793_Calibrate(AD7793_CH_AIN2P_AIN2M, CAL_INT_ZERO_MODE);
   AD7793_Calibrate(AD7793_CH_AIN2P_AIN2M, CAL_INT_FULL_MODE);
   AD7793_Calibrate(AD7793_CH_AIN3P_AIN3M, CAL_INT_ZERO_MODE);
   AD7793_Calibrate(AD7793_CH_AIN3P_AIN3M, CAL_INT_FULL_MODE);

#if(USE_IOUT2 == YES)
   ui32result = AD7793_Scan(SINGLE_CONV, AD7793_CH_AIN3P_AIN3M);
   i32voltage = AD7793_ConvertToVolts(ui32result);
   iout2_calibration = u32voltage / (float)5000;
#endif
}


/**
   @brief Calculate temperature value

   @return float - temperature value
**/
float CN0326_CalculateTemp(void)
{

   static float temp, res, f32current;

   uint32_t ui32adcValue;
   int32_t i32voltage;

#if(USE_IOUT2 == YES)                           /* Check which excitation current to use */
   f32current = iout2_calibration;
#else
   f32current = I_EXC ;
#endif

   ui32adcValue = AD7793_Scan(SINGLE_CONV, AD7793_CH_AIN2P_AIN2M);         /* Read ADC output value */

   i32voltage = AD7793_ConvertToVolts(ui32adcValue);        /* Convert ADC output value to voltage */

   res = i32voltage / (float)f32current;                    /* Calculate RTD resistance */

   temp = ((res - RMIN) / (TEMP_COEFF * RMIN));             /* Calculate temperature value */

   return temp;
}


/**
   @brief Calculate pH value

   @return float - pH value
**/
float CN0326_CalculatePH(void)
{
   float temp, ph;

   uint32_t ui32adcValue;
   int32_t i32voltage;

   temp = CN0326_CalculateTemp();                       /* Calculate temperature */

   ui32adcValue = AD7793_Scan(SINGLE_CONV, AD7793_CH_AIN1P_AIN1M);         /* Read ADC output value */

   i32voltage = AD7793_ConvertToVolts(ui32adcValue);        /* Convert ADC output value to voltage */

   ph = PH_ISO - (((double)((i32voltage - TOLERANCE) * FARADAY_CONST)) / (PH_CONST * AVOGADRO_NUMBER * (temp + K_DEGREES))); /* Calculate pH value */

   return ph;
}




#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
   @brief Display info for <help> command

   @param args - pointer to the arguments on the command line.

   @return none
**/
void CN0326_CmdHelp(uint8_t *args)
{
   printf("\n");
   printf("Available commands:\n");
   printf(" help                   - Display available commands\n");
   printf(" calibrate <ch>         - Calibrate selected channel or all channels. \n");
   printf("                          <ch> = AIN1, AIN2, AIN3 or all \n");
   printf(" temp                   - Display temperature value\n");
   printf(" ph                     - Display pH value\n");
   printf(" reset                  - Reset ADC converter\n");

}

/**
   @brief Display info for <temp> command

   @param args - pointer to the arguments on the command line.

   @return none
**/
void CN0326_CmdTemp(uint8_t *args)
{
   float temp;

   temp = CN0326_CalculateTemp();

   printf("Temperature = %.2f[˚C]\n", temp);

}


/**
   @brief Display info for <ph> command

   @param args - pointer to the arguments on the command line.

   @return none
**/
void CN0326_CmdPH(uint8_t *args)
{
   float ph;

   ph = CN0326_CalculatePH();

   printf("pH = %.2f\n", ph);

}


/**
   @brief Display info for <calibrate> command

   @param args - pointer to the arguments on the command line.

   @return none
**/
void CN0326_CmdCalibration(uint8_t *args)
{

   uint8_t  *p = args;
   char     arg[5];
   uint8_t ui8channel;

   while (*(p = CN0326_FindArgv(p)) != '\0') {         /* Check if this function gets an argument */
      CN0326_GetArgv(arg, p);                        /* Save channel parameter */
   }

   if(strncmp(arg, "AIN1", 5) == 0) {

      AD7793_Calibrate(AD7793_CH_AIN1P_AIN1M, CAL_INT_ZERO_MODE);
      AD7793_Calibrate(AD7793_CH_AIN1P_AIN1M, CAL_INT_FULL_MODE);

      timer_sleep(300);

      printf("Calibration completed for %s channel!\n", arg);

   } else if(strncmp(arg, "AIN2", 5) == 0) {

      AD7793_Calibrate(AD7793_CH_AIN2P_AIN2M, CAL_INT_ZERO_MODE);
      AD7793_Calibrate(AD7793_CH_AIN2P_AIN2M, CAL_INT_FULL_MODE);

      timer_sleep(300);

      printf("Calibration completed for %s channel!\n", arg);

   } else if(strncmp(arg, "AIN3", 5) == 0) {

      AD7793_Calibrate(AD7793_CH_AIN3P_AIN3M, CAL_INT_ZERO_MODE);
      AD7793_Calibrate(AD7793_CH_AIN3P_AIN3M, CAL_INT_FULL_MODE);

      timer_sleep(300);

      printf("Calibration completed for %s channel!\n", arg);

   } else if(strncmp(arg, "all", 4) == 0) {

      for(ui8channel = 0; ui8channel <= 2; ui8channel++) {

         AD7793_Calibrate(ui8channel, CAL_INT_ZERO_MODE);
         AD7793_Calibrate(ui8channel, CAL_INT_FULL_MODE);
      }

      timer_sleep(300);

      printf("Calibration completed for %s channels!\n", arg);

   } else {

      printf("Incorrect channel!Try again!\n");
   }


}


/**
   @brief Finds the next command line argument

   @param args - pointer to the arguments on the command line.

   @return pointer to the next argument.

**/
uint8_t *CN0326_FindArgv(uint8_t *args)
{
   uint8_t  *p = args;
   int         fl = 0;

   while (*p != 0) {
      if ((*p == _SPC)) {
         fl = 1;

      } else {
         if (fl) {
            break;
         }
      }

      p++;
   }

   return p;
}

/**
   @brief Separates a command line argument

    @param dst - pointer to a buffer where the argument will be copied
    @param args - pointer to the current position of the command line .

   @return none

**/
void CN0326_GetArgv(char *dst, uint8_t *args)
{
   uint8_t  *s = args;
   char     *d = dst;

   while (*s) {
      if (*s == _SPC) {
         break;
      }

      *d++ = *s++;
   }

   *d = '\0';

}


/**
   @brief Display info for <reset> command

   @param args - pointer to the arguments on the command line.

   @return none
**/
void CN0326_CmdReset(uint8_t *args)
{

   AD7793_Reset();

   timer_sleep(500);

   printf("AD7793 reset completed!\n");

}
#pragma GCC diagnostic pop


/**
   @brief Command line process function

   @return none
**/
void CN0326_CmdProcess(void)
{
   cmdFunc   func;

   if (uart_cmd == UART_TRUE) {           /* Check if <ENTER> key was pressed */

      func = CN0326_FindCommand((char *)uart_rx_buffer);            /* Find needed function based on typed command */

      if (func) {                                                      /* Check if there is a valid command */
         printf("\n");
         (*func)(&uart_rx_buffer[2]);                                  /* Call the desired function */

      } else if (strlen((char *)uart_rx_buffer) != 0) {                /* Check if there is no match for typed command */
         printf("\n");
         printf("Unknown command!");                                   /* Display a message for unknown command */
         printf("\n");
      }

      uart_cmd = UART_FALSE;                                            /* Prepare for next <ENTER> */
      CN0326_CmdPrompt();
   }
}


/**
   @brief Command line prompt

   @return int - UART status: UART_SUCCESS or error status
**/
int CN0326_CmdPrompt(void)
{
   int res;
   static uint8_t count = 0;

   res = UART_WriteChar(_CR, UART_WRITE_NO_INT);

   if (res == UART_SUCCESS) {
      res = UART_WriteChar(_LF, UART_WRITE_NO_INT);
   }

   if(count == 0) {                  /* Check first <ENTER> is pressed after reset */
      printf("\tWelcome to CN0326 application!\n");
      printf("Type <help> to see available commands...\n");
      printf("\n");
      count++;
   }

   if (res == UART_SUCCESS) {
      UART_WriteChar(':', UART_WRITE_NO_INT);
   }

   uart_rcnt = 0;

   return res;
}

/**
   @brief Find available commands

   @param cmd - command to search

   @return cmdFunc - return the specific function for available command or NULL for invalid command
**/
cmdFunc CN0326_FindCommand(char *cmd)
{
   cmdFunc func = NULL;
   int i = 0;

   while (CmdFun[i] != NULL) {
      if (strncmp(cmd, CmdCommands[i], 6) == 0) {
         func = CmdFun[i];
         break;
      }

      i++;
   }

   return func;
}


/**
   @brief Internal interrupt handler for UART

   @return none

**/
void CN0326_Interrupt(void)
{

   unsigned short  status;
   char c;

   status = UrtIntSta(pADI_UART);

   if (status & COMIIR_NINT) {
      return;
   }

   switch (status & COMIIR_STA_MSK) {

   case COMIIR_STA_RXBUFFULL:

      UART_ReadChar(&c);

      switch(c) {

      case _CR:
         uart_cmd = UART_TRUE;
         break;

      default:
         uart_rx_buffer[uart_rcnt++] = c;

         if (uart_rcnt == UART_RX_BUFFER_SIZE) {
            uart_rcnt--;
            UART_WriteChar(_BS, UART_WRITE_IN_INT);
         }

         UART_WriteChar(c, UART_WRITE_IN_INT);
      }

      uart_rx_buffer[uart_rcnt] = '\0';
      break;

   case COMIIR_STA_TXBUFEMPTY:
      if (uart_tcnt) {
         uart_tbusy = UART_TRUE;
         uart_tcnt--;
         UART_WriteChar(uart_tx_buffer[uart_tpos++], UART_WRITE);

         if (uart_tpos == UART_TX_BUFFER_SIZE) {
            uart_tpos = 0;
         }

      } else {
         uart_tbusy = UART_FALSE;
      }

      break;

   default:
      ;
   }

}
