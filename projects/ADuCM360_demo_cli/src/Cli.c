/**
******************************************************************************
*   @file     Cli.c
*   @brief    Source file for CLI demo.
*   @version  V0.1
*   @author   ADI
*   @date     August 2015
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
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include <ADuCM360.h>
#include <DioLib.h>

#include "Cli.h"
#include "Timer.h"
#include "Communication.h"

/************************* Variable Definitions ******************************/

/* Available commands */
char *kwnUrtMonCmd[] = {
   "help",
   "version",
   "dump",
   "reset",
   ""
};

/* Functions for available commands */
msUartCPFun urtMonCmdFun[] = {
   Cli_CmdHelp,
   Cli_CmdVersion,
   Cli_CmdDump,
   Cli_CmdReset,
   NULL
};


/************************* Functions Definitions ******************************/


/**
   @brief Display the current version of the software

   @param args - pointer to the arguments on the command line.

   @return UART_SUCCESS or error code.

**/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
int Cli_CmdVersion(uint8_t *args)
{
   UART_WriteString("SW version of CLI Demo:");
   UART_WriteString(VERSION);
   UART_WriteString("\n");
   return UART_SUCCESS;
}
#pragma GCC diagnostic pop

/**
   @brief Reset function
   @param args - pointer to the arguments on the command line.

   @return UART_SUCCESS or error code.

**/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
int Cli_CmdReset(uint8_t *args)
{
   //exit(1);
   NVIC_SystemReset();
   return UART_SUCCESS;
}
#pragma GCC diagnostic pop

/**
   @brief Display usage information

   @param args - pointer to the arguments on the command line.

   @return UART_SUCCESS or error code.

**/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
int Cli_CmdHelp(uint8_t *args)
{
   UART_WriteString("\n");
   UART_WriteString("Available commands:\r\n");
   UART_WriteString(" help                   - Display available commands\r\n");
   UART_WriteString(" version                - SW version\r\n");
   UART_WriteString(" dump [begaddr][,count] - Dumps the memory\r\n");
   UART_WriteString(" reset                  - Reset application\r\n");
   return UART_SUCCESS;
}
#pragma GCC diagnostic pop

/**
   @brief Dumps the memory content

   @param args - pointer to the arguments on the command line.

   @return UART_SUCCESS or error code.

**/
int Cli_CmdDump(uint8_t *args)
{
   int         argc = 1;
   uint8_t  *p = args;
   char     arg[UART_CMD_USUALMAXLEN];
   static uint32_t   begaddr = 0, count = 16, ix;
   int         fl = 0;

   while (*(p = Cli_FindNextArgv(p)) != '\0') {
      Cli_GetArgv(arg, p);

      switch(argc) {
      case 1:
         begaddr = strtol(arg, NULL, 16);
         break;

      case 2:
         count = strtol(arg, NULL, 16);

         if (count > 0x40) {
            count = 0x40;
         }

         break;

      default:
         ;
      }

      argc++;
   }

   Cli_Printf("HEX dump of the memory (0x%02X bytes from 0x%08X):\r\n", count, begaddr);
   fl = 1;

   for (ix = begaddr; ix < (begaddr + count); ix += 4) {
      if ((ix != begaddr) && (ix % 16 == 0)) {
         fl = 1;
         Cli_Printf("\r\n");
      }

      if (fl) {
         Cli_Printf(" %08X: ", ix);
         fl = 0;
      }

      Cli_Printf("%08X ", *((uint32_t *)ix));
   }

   return UART_SUCCESS;
}

/**
   @brief Finds the next command line argument

   @param args - pointer to the arguments on the command line.

   @return pointer to the next argument.

**/
uint8_t *Cli_FindNextArgv(uint8_t *args)
{
   uint8_t  *p = args;
   int         fl = 0;

   while (*p != 0) {
      if ((*p == SPC_) || (*p == TAB_)) {
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

   @return UART_SUCCESS or error code.

**/
int Cli_GetArgv(char *dst, uint8_t *args)
{
   uint8_t  *s = args;
   char     *d = dst;

   while (*s) {
      if (Cli_IsSeparator(*s)) {
         break;
      }

      *d++ = *s++;
   }

   *d = '\0';

   return UART_SUCCESS;
}

/**
   @brief Checks a character for being a separators in the command line input

   @param c - the character to be checked.

   @return  UART_TRUE or UART_FALSE
**/

int Cli_IsSeparator(uint8_t c)
{
   return ((c == SPC_) || (c == TAB_) || (c == ',')) ? UART_TRUE : UART_FALSE;
}

/**
   @brief Write the prompt

   @return UART_SUCCESS or error code.
**/
int Cli_Prompt(void)
{
   int res;
   static unsigned int count = 0;

   res = UART_WriteChar(CR_, UART_WRITE_NO_INT);

   if (res == UART_SUCCESS) {
      res = UART_WriteChar(LF_, UART_WRITE_NO_INT);
   }

   if(count == 0) {
      UART_WriteString("\t Welcome to Command Line Interpreter!!!\r\n");
      UART_WriteString("\r\n");
      UART_WriteString("\r\n");
      UART_WriteString(">>Type in <help> to see available commands.\r");
      UART_WriteString("\r\n");
      count++;
   }

   if (res == UART_SUCCESS) {
      UART_WriteString(">>");
   }

   uart_rcnt = 0;

   return res;
}

/**
   @brief Command line interpreter process function

**/
void Cli_Process(void)
{
   msUartCPFun   fp;

   if (uart_cmd == UART_TRUE) {
      fp = Cli_FindCommandFunction((char *)uart_rx_buffer);

      if (fp) {
         UART_WriteString("\r\n");
         (*fp)(&uart_rx_buffer[UART_CMD_LENGTH]);

      } else if (strlen((char *)uart_rx_buffer) != 0) {
         UART_WriteString("\r\nUnknown command!");
      }

      uart_cmd = UART_FALSE;
      Cli_Prompt();
   }
}

/**
   @brief Looks and finds the function for the command string

   @param cmd - pointer to the command string.

   @return Pointer to the function or NULL if no such command is found.
**/
msUartCPFun Cli_FindCommandFunction(char *cmd)
{
   msUartCPFun fp = NULL;
   int         i;

   i = 0;

   while (urtMonCmdFun[i] != NULL) {
      if (strncmp(cmd, kwnUrtMonCmd[i], UART_CMD_LENGTH) == 0) {
         fp = urtMonCmdFun[i];
         break;
      }

      i++;
   }

   return fp;
}

/**
   @brief Write string to output

   @param fmt a pointer to a formatting string.
    @param ... variable number and list of arguments

**/
void Cli_Printf(const char *fmt, ...)
{
   char buff[256];

   va_list args;
   va_start (args, fmt);
   vsprintf (buff, fmt, args);
   va_end (args);
   UART_WriteString(buff);
}



/**
   @brief CLI interrupt handler

**/
void Cli_Int_Handler(void)
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

      if (c == CTRL_C_) {
         uart_ctrlc = UART_TRUE;

      } else if (uart_cmd == UART_FALSE) {
         switch(c) {
         case SP_:
         case BS_:
            if (uart_rcnt) {
               uart_rcnt--;
               uart_rx_buffer[uart_rcnt] = 0;
               UART_WriteChar(c, UART_WRITE_IN_INT);
               UART_WriteChar(' ', UART_WRITE_IN_INT);
               UART_WriteChar(c, UART_WRITE_IN_INT);
            }

            break;

         case CR_:
            uart_cmd = UART_TRUE;
            break;

         default:
            uart_rx_buffer[uart_rcnt++] = c;

            if (uart_rcnt == UART_RX_BUFFER_SIZE) {
               uart_rcnt--;
               UART_WriteChar(BS_, UART_WRITE_IN_INT);
            }

            UART_WriteChar(c, UART_WRITE_IN_INT);
         }
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

