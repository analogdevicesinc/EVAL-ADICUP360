#ifndef COMMAND_H_
   #define COMMAND_H_

   typedef void (*cmdFunc)(uint8_t *);

   void Command_Help(uint8_t *args);
   void Command_Display(uint8_t *args);
   void Command_Cal_Light(uint8_t *args);
   void Command_Cal_pH(uint8_t *args);
   void Command_Test_Red(uint8_t *args);
   void Command_Test_Green(uint8_t *args);
   void Command_Test_Blue(uint8_t *args);
   void Command_Set_Red(uint8_t *args);
   void Command_Set_Green(uint8_t *args);
   void Command_Set_Blue(uint8_t *args);
   void Command_Acquire(uint8_t *args);
   void Command_Start_Ctrl(uint8_t *args);
   void Command_Stop_Ctrl(uint8_t *args);
   void Command_Rest(uint8_t *args);
   uint8_t *Command_FindArgv(uint8_t *args);
   void Command_GetArgv(char *dst, uint8_t *args);
   void Command_ControlLux(void);
   void Command_Prompt(void);
   void Command_Process(void);
   cmdFunc Command_FindCommand(char *cmd);

   extern uint8_t display_enable;
   extern uint8_t start_acquire;

#endif
