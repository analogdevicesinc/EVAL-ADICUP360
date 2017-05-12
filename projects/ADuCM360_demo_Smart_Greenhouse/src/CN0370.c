/***************************** Library Include Files **************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/***************************** Source Include Files ***************************/
#include "ADuCM360.h"
#include "Communication.h"
#include "Timer.h"
#include "CN0397.h"

/***************************** Class Variables ********************************/
unsigned int set_red = 0;
unsigned int set_ble = 0;
unsigned int set_grn = 0;

/* Control System Variable */
// 0 - No control
// 1 - bang-bang control
// 2 - pid control
uint8_t control_system = 0;

uint16_t code_led[3] = {0, 0, 0};
float desired_lux[3] = {0, 0, 0};

void CN0370_SetRedLux(float temp)
{
   desired_lux[0] = temp;
}

void CN0370_SetGrnLux(float temp)
{
   desired_lux[1] = temp;
}

void CN0370_SetBleLux(float temp)
{
   desired_lux[2] = temp;
}

void CN0370_SetControlMode(uint8_t temp)
{
   switch(temp)
   {
      case 0:
         control_system = 1;
         printf("Proportional Control\n");
         break;
      case 1:
         control_system = 2;
         break;
      default:
         control_system = 0;
         break;
   }
}

void CN0370_SetRED(unsigned int temp)
{
   if (control_system == 0)
   {
      set_red = temp; // convert string to float
      SPI0_Write(set_red, RED_LED);
   }
   else
   {
      printf("Cannot adjust LED while control system is active\n");
   }
}

void CN0370_SetGRN(unsigned int temp)
{
   if (control_system == 0)
   {
      set_grn = temp; // convert string to float
      SPI0_Write(set_grn, GRN_LED);
   }
   else
   {
      printf("Cannot adjust LED while control system is active\n");
   }
}

void CN0370_SetBLE(unsigned int temp)
{
   if (control_system == 0)
   {
      set_ble = temp; // convert string to float
      SPI0_Write(set_ble, BLE_LED);
   }
   else
   {
      printf("Cannot adjust LED while control system is active\n");
   }
}

void CN0370_Zero(void)
{
   SPI0_Write(0, RED_LED);
   SPI0_Write(0, BLE_LED);
   SPI0_Write(0, GRN_LED);
}

