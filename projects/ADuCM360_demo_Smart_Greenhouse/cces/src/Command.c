/***************************** Library Include Files **************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************** Source Include Files ***************************/
#include "ADuCM360.h"
#include "Communication.h"
#include "CN0397.h"
#include "CN0398.h"
#include "CN0370.h"
#include "Command.h"

/***************************** Class Variables ********************************/
char *CmdCommands[] =
{
   "help",
   "display",
   "acquire",
   "cal_pd",
   "cal_ph",
   "red_test",
   "green_test",
   "blue_test",
   "set_red",
   "set_green",
   "set_blue",
   "start",
   "stop",
   "rest",
   ""
};

cmdFunc CmdFun[] = {
   Command_Help,
   Command_Display,
   Command_Acquire,
   Command_Cal_Light,
   Command_Cal_pH,
   Command_Test_Red,
   Command_Test_Green,
   Command_Test_Blue,
   Command_Set_Red,
   Command_Set_Green,
   Command_Set_Blue,
   Command_Start_Ctrl,
   Command_Stop_Ctrl,
   Command_Rest,
   NULL
};

uint8_t display_enable  = 0;
uint8_t start_acquire   = 0;

void Command_Help(uint8_t *args)
{
   printf("\n");
   printf("Available commands:\n");
   printf("\n");
   printf("display                - Continuously display all sensor data (Press <ENTER> to stop)\n");
   printf("acquire                - Continuously acquire all sensor data\n");
   printf("rest                   - Stop acquiring sensor data\n");
   printf("cal_pd                 - Calibrate CN0397 ADC for photodiode zero-scale initialization\n");
   printf("cal_ph                 - Calibrate CN0398 ADC for voltage to pH conversion\n");
   printf("red_test <0:65535>     - Perform functionality test for CN0370 with red LED\n");
   printf("green_test <0:65535>   - Perform functionality test for CN0370 with green LED\n");
   printf("blue_test <0:65535>    - Perform functionality test for CN0370 with blue LED\n");
   printf("set_red <0:120000>     - Set desired red light intensity value to maintain in lux\n");
   printf("set_green <0:120000>   - Set desired green light intensity value to maintain in lux\n");
   printf("set_blue <0:120000>    - Set desired blue light intensity value to maintain in lux\n");
   printf("start                  - Start Proportional Control System for CN0370\n");
   printf("stop                   - Stop Proportional Control System for CN0370\n");
   printf("\n");
}

void Command_Display(uint8_t *args)
{
   display_enable = 1;
   //CN0397_DisplayData();
   //CN0398_display_data();
}

void Command_Cal_Light(uint8_t *args)
{
   //CN0397_Init();
   CN0397_StartCal();
}

void Command_Cal_pH(uint8_t *args)
{
   //CN0398_Setup();
   CN0398_calibrate();
}

void Command_Test_Red(uint8_t *args)
{
   uint8_t  *p = args;
   char     arg[7];
   unsigned int temp;

   while (*(p = Command_FindArgv(p)) != '\0')
   {
         Command_GetArgv(arg, p);
   }
   temp = atoi(arg);
   CN0370_SetRED(temp);

}

void Command_Test_Green(uint8_t *args)
{
   uint8_t  *p = args;
   char     arg[7];
   unsigned int temp;

   while (*(p = Command_FindArgv(p)) != '\0')
   {
         Command_GetArgv(arg, p);
   }
   temp = atoi(arg);
   CN0370_SetGRN(temp);
}

void Command_Test_Blue(uint8_t *args)
{
   uint8_t  *p = args;
   char     arg[7];
   unsigned int temp;

   while (*(p = Command_FindArgv(p)) != '\0')
   {
         Command_GetArgv(arg, p);
   }
   temp = atoi(arg);
   CN0370_SetBLE(temp);
}

void Command_Set_Red(uint8_t *args)
{
   uint8_t  *p = args;
   char     arg[7];
   float    temp;

   while (*(p = Command_FindArgv(p)) != '\0')
   {
         Command_GetArgv(arg, p);
   }
   temp = atof(arg);
   CN0370_SetRedLux(temp);
}

void Command_Set_Green(uint8_t *args)
{
   uint8_t  *p = args;
   char     arg[7];
   float    temp;

   while (*(p = Command_FindArgv(p)) != '\0')
   {
         Command_GetArgv(arg, p);
   }
   temp = atof(arg);
   CN0370_SetGrnLux(temp);
}

void Command_Set_Blue(uint8_t *args)
{
   uint8_t  *p = args;
   char     arg[7];
   float    temp;

   while (*(p = Command_FindArgv(p)) != '\0')
   {
         Command_GetArgv(arg, p);
   }
   temp = atof(arg);
   CN0370_SetBleLux(temp);
}

void Command_Acquire(uint8_t *args)
{
   start_acquire = 1;
}

void Command_Start_Ctrl(uint8_t *args)
{
   uint8_t  *p = args;
   char     arg[7];
   float    temp;

   while (*(p = Command_FindArgv(p)) != '\0')
   {
         Command_GetArgv(arg, p);
   }
   temp = atoi(arg);

   CN0370_SetControlMode(temp);
}

void Command_Stop_Ctrl(uint8_t *args)
{
   control_system = 0;
}

void Command_Rest(uint8_t *args)
{
   start_acquire = 0;
}

uint8_t *Command_FindArgv(uint8_t *args)
{
   uint8_t  *p = args;
   int fl = 0;

   while (*p != 0)
   {
      if ((*p == _SPC))
      {
         fl = 1;

      }
      else
      {
         if (fl)
         {
            break;
         }
      }

      p++;
   }

   return p;
}

void Command_GetArgv(char *dst, uint8_t *args)
{
   uint8_t  *s = args;
   char     *d = dst;

   while (*s)
   {
      if (*s == _SPC)
      {
         break;
      }

      *d++ = *s++;
   }

   *d = '\0';
}

void Command_ControlLux(void)
{
   uint8_t channel, rgbChannel;
   uint16_t desired_adc;

   switch(control_system)
   {
      case 0:
         break;

      case 1:
         for (channel = 0; channel < CHANNELS; channel++)
         {
               rgbChannel = Channels[channel];
               desired_adc = (uint16_t) (desired_lux[rgbChannel] / Lux_LSB[rgbChannel]);
               code_led[rgbChannel] += 0.5 * (desired_adc - cn0397_adcValue[rgbChannel]);
               SPI0_Write(code_led[rgbChannel], rgbChannel);
         }
         break;

      case 2:
         break;
   }
}

void Command_Prompt(void)
{
   uart_cmd = UART_FALSE;
   uart_rcnt = 0;
   UART_WriteChar('\n');
   UART_WriteChar('>');
}

void Command_Process(void)
{
   cmdFunc   func;

   func = Command_FindCommand((char *)uart_rx_buffer);             /* Find needed function based on typed command */

   if (func)
   {                                                              /* Check if there is a valid command */
      printf("\n");

      (*func)(&uart_rx_buffer[2]);                             /* Call the desired function */
   }
   else if (strlen((char *)uart_rx_buffer) != 0)
   {
      /* Check if there is no match for typed command */
      printf("\n");
      printf("Unknown command!");                              /* Display a message for unknown command */
      printf("\n");
   }
   else
   {
      display_enable = 0;
   }
}

cmdFunc Command_FindCommand(char *cmd)
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
