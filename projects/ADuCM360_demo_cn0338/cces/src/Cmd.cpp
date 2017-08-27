/**
******************************************************************************
*   @file     Cmd.cpp
*   @brief    Source file for command line interpreter
*   @version  V0.1
*   @author   ADI
*   @date     February 2016
*  @par Revision History:
*  - V0.1, February 2016: initial version.
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
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <cassert>

#include <Cmd.h>

#include <ADuCM360.h>
#include <AdcLib.h>

#include <cstdio>
#include <cstring>

#include "CN0338.h"
#include <Communication.h>
#include <ADC.h>
#include <Flash.h>

/****************************** Global variables *****************************/

time_t start_time;
CCmdPrompt Prompt;
CCmdStart CmdStart;
CCmdHelp CmdHelp;
CCmdRun CmdRun;
CmdTarget *pActiveTarget = &CmdStart;
CCmdResetToDefault CmdResetToDefault;

CMD_MAP(5, help, &CmdHelp, "  Display available commands\t\t");
CMD_MAP(run, &CmdRun, "  Starts CN0338 measurements\t\t");
CMD_MAP(resetTodefault, &CmdResetToDefault, "  Reset system settings to default values");


/****************************** Global functions *****************************/

void (* volatile msg_queue[MSG_QUEUE_SIZE])(void);

/**
   @brief Display prompt (when <ENTER> is pressed)

   @return none
**/
void CCmdPrompt::on_Enter(void)
{
   char cmd[CMD_BUFFER_SIZE] = { };

   scanf("%s", cmd);

   if (strlen(cmd) == 0) {
      printf("CN0338:>");

      fflush(stdout);
      scanf("%*[^][]");

      return;
   }

   for (const CmdTable *p = __cmd_table_start__; p < __cmd_table_end__; ++p) {
      if (strcmp(cmd, p->p_cmd) == 0) {
         pActiveTarget = p->p_target;
         p->p_target->on_Enter();
         return;
      }
   }

   scanf("%*[^][]");

   printf("\nCommand '%s' is not found!\r\nPlease enter 'help'!\r\n\nCN0338:>", cmd);
   fflush(stdout);
}

/**
   @brief Main application (when <ENTER> is pressed)

   @return none
**/
void CmdTarget::on_Enter(void)
{
   /* Nothing to do */
}

/**
   @brief Main application (when <Ctrl + 'c'> combination is pressed)

   @return none
**/
void CmdTarget::on_Ctrl_c(void)
{
   pActiveTarget = &Prompt;
   scanf("%*[^][]");
   puts("\r\n");
   pActiveTarget->on_Enter();
}

/**
   @brief Main application (when NDIR occur)

   @return none
**/
void CmdTarget::on_NDIR(void)
{
   /* Nothing to do */
}

/**
   @brief Help command (when <ENTER> is pressed)

   @return none
**/
void CCmdHelp::on_Enter(void)
{

   printf("\n");
   printf("\t|\t \033[1mCOMMAND\033[0m\t|\t\t\033[1mDESCRIPTION\033[0m\t\t\t|\r\n");

   printf("\t+-----------------------+----------------------------------------------+\r\n");

   for (CmdTable *p = (&CMD_OBJ(help)); p < __cmd_table_end__; ++p) {
      printf("\t|     %s", p->p_cmd);

      if(strlen(p->p_cmd) < 10) {

         printf("\t");
      }

      printf("\t|%s\t|\r\n", p->p_help);
   }

   printf("\t+-----------------------+----------------------------------------------+\r\n");

   puts("\n\033[4mNOTE\033[0m: To abort a running command just use the combination '\033[1mCtrl + c\033[0m'.\r\n\n");

   fflush(stdout);

   scanf("%*[^][]");

   pActiveTarget = &Prompt;
   pActiveTarget->on_Enter();
}

/**
   @brief Read command

   @return none
**/
void Cmd_ReadData(void)
{
   while (uart_rx_head != uart_rx_tail) {
      switch (uart_rx_queue[uart_rx_head]) {
      case ' ' ... '~':
         cmd_buffer[cmd_tail] = uart_rx_queue[uart_rx_head];

         putchar(cmd_buffer[cmd_tail]);

         if (cmd_tail != (CMD_BUFFER_SIZE - 1)) {
            ++cmd_tail;

         } else {
            cmd_tail = 0;
         }

         if (cmd_tail == cmd_head) {
            puts("\r\nYou typed too much!\r");
            printf("Please re-enter: ");
         }

         break;

      case '\r':
      case '\n':
         puts("\r");
         pActiveTarget->on_Enter();
         break;

      case '':
      case '':
         if (cmd_tail != cmd_head) {
            printf(" ");

            if (cmd_tail != 0) {
               --cmd_tail;

            } else {
               cmd_tail = (CMD_BUFFER_SIZE - 1);
            }
         }

         break;

      case '':
         pActiveTarget->on_Ctrl_c();
         break;

      default:
         break;
      }

      fflush(stdout);

      if (uart_rx_head == (UART_RX_QUEUE_SIZE - 1)) {
         uart_rx_head = 0;

      } else {
         ++uart_rx_head;
      }
   }
}

/**
   @brief Start command (when <ENTER> is pressed)

   @return none
**/
void CCmdStart::on_Enter(void)
{
   char cmd[CMD_BUFFER_SIZE] = { };
   scanf("%s", cmd);


   puts("\tWELCOME TO EVAL-CN0338-ARDZ\r\n");
   printf("\n");

   if (!Flash_VerifyOptions()) {
      SOptions options;
      Flash_SetOptions(options);
      puts("All environment variables are set to default!\r");
      puts("For first use, please calibrate!!!\r\n\r");

   }

   puts("Enter 'help' to see available commands.\r\n\r");

   CN0338_InitNDIR();

   time(&start_time);

   pActiveTarget = &Prompt;
   pActiveTarget->on_Enter();


   fflush(stdout);
   scanf("%*[^][]");
}

/**
   @brief Run command(when <ENTER> is pressed)

   @return none
**/
void CCmdRun::on_Enter(void)
{
   puts("\n\033[4mNOTE\033[0m: To abort this command just use the combination '\033[1mCtrl + c\033[0m'.\r\n");

   fflush(stdout);

   scanf("%*[^][]");
}

/**
   @brief Run command(when NDIR occur)

   @return none
**/
void CCmdRun::on_NDIR(void)
{

   printf("\nCO2 concentration = %f%c\r\n", CN0338_GetConcentration(), 0x25);
   printf("Temperature = %f˚C\r\n", CN0338_GetCelsius());

   double dadc1high_data = adc1high_data, dadc1low_data = adc1low_data;
   double dadc0high_data = adc0high_data, dadc0low_data = adc0low_data;

   double dFull = 223696.213333333333;

   printf("REF high voltage = %fmv\r\n", dadc1high_data / dFull);
   printf("REF low voltage = %fmv\r\n", dadc1low_data / dFull);
   printf("    REF diff voltage = %fmv\r\n", (dadc1high_data - dadc1low_data) / dFull);

   printf("ACT high voltage = %fmv\r\n", dadc0high_data / dFull);
   printf("ACT low voltage = %fmv\r\n", dadc0low_data / dFull);
   printf("    ACT diff voltage = %fmv\r\n", (dadc0high_data - dadc0low_data) / dFull);

   float I = (float) (adc0high_data - adc0low_data) / (float) (adc1high_data - adc1low_data);
   float fa = 1 - (I / Flash_GetOptions().zero);
   printf("FA = %f\r\n", fa);

   fflush(stdout);
}

/**
   @brief Run command(when <Ctrl + 'c'> combination is pressed)

   @return none
**/
void CCmdRun::on_Ctrl_c(void)
{
   scanf("%*[^][]");
   printf("\r\n");
   pActiveTarget = &Prompt;
   pActiveTarget->on_Enter();

}

/**
   @brief Reset to default values command (when <ENTER> is pressed)

   @return none
**/
void CCmdResetToDefault::on_Enter(void)
{
   SOptions defaultOptions;

   UART_Init(defaultOptions.baud_rate);

   int AF, SF;

   auto frequency = ADC_FindFactor(AF, SF, defaultOptions.adc_frequency);

   pADI_ADC0->FLT = ADC0FLT_CHOP_ON | ADC0FLT_RAVG2_ON | ADC0FLT_SINC4EN_DIS | (AF << 8) | ADC0FLT_NOTCH2_EN | SF;
   pADI_ADC1->FLT = ADC1FLT_CHOP_ON | ADC1FLT_RAVG2_ON | ADC1FLT_SINC4EN_DIS | (AF << 8) | ADC1FLT_NOTCH2_EN | SF;

   defaultOptions.adc_frequency = frequency;

   puts("\nAll settings are reset to default values:\r\n");

   printf("\t- Baud rate =  %dHz\r\n", defaultOptions.baud_rate);

   printf("\t- NDIR light source frequency = %fHz\r\n", defaultOptions.ndir_frequency);

   printf("\t- NDIR rising edge time = %ums\r\n", defaultOptions.rising_ms);

   printf("\t- NDIR falling edge time = %ums\r\n", defaultOptions.falling_ms);

   printf("\t- ADC sample frequency = %fHz, AF=%i, SF=%i\r\n", frequency, AF, SF);

   printf("\t- Coefficient 'ZERO' = %f\r\n", defaultOptions.zero);

   printf("\t- Coefficient 'SPAN' = %f\r\n", defaultOptions.span);

   printf("\t- Coefficient 'b' = %f\r\n", defaultOptions.b);

   printf("\t- Coefficient 'c' = %f\r\n", defaultOptions.c);

   printf("\t- Calibrated temperature 'k' = %f˚C\r\n", CN0338_KelvinToCelsius(defaultOptions.k));

   printf("\n");

   Flash_SetOptions(defaultOptions);

   fflush(stdout);

   scanf("%*[^][]");

   pActiveTarget = &Prompt;
   pActiveTarget->on_Enter();
}

volatile auto msg_head = 0, msg_tail = 0;

/**
   @brief Set message to be sent

   @param *p_fun function to be called

   @return none
**/
void Cmd_SetMsg(void (*p_fun)(void))
{
   __disable_irq();

   msg_queue[msg_tail] = p_fun;

   if (msg_tail == (MSG_QUEUE_SIZE - 1)) {
      msg_tail = 0;

   } else {
      ++msg_tail;
   }

   __enable_irq();

   assert(msg_tail != msg_head);
}

/**
   @brief Read message (command)

   @return none
**/
void Cmd_ReadMsg(void)
{
   while (msg_head == msg_tail)
      ;

   (*msg_queue[msg_head])();

   if (msg_head == (MSG_QUEUE_SIZE - 1)) {
      msg_head = 0;

   } else {
      ++msg_head;
   }
}
