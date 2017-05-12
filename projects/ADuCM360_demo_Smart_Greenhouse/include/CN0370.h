#ifndef _CN0370_H_
   #define _CN0370_H_

   void CN0370_SetRedLux(float temp);
   void CN0370_SetGrnLux(float temp);
   void CN0370_SetBleLux(float temp);
   void CN0370_SetControlMode(uint8_t temp);

   // Test Functions
   void CN0370_SetRED(unsigned int temp);
   void CN0370_SetGRN(unsigned int temp);
   void CN0370_SetBLE(unsigned int temp);

   void CN0370_Zero(void);

   extern uint8_t control_system;

   extern float desired_lux[3];
   extern uint16_t code_led[3];

#endif
