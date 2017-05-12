/***************************** Library Include Files **************************/
#include <stdio.h>
#include <stdlib.h>

/***************************** Source Include Files ***************************/
#include "lcd.h"
#include "Communication.h"
#include "Timer.h"

/***************************** Class Variables ********************************/
uint8_t _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
uint8_t _displaycontrol;
uint8_t _displaymode;

uint8_t _rs_pin         = 15;
uint8_t _rw_pin         = 14;
uint8_t _enable_pin     = 13;
uint8_t _data_pins[4]   = {12, 11, 10, 9};
uint8_t _button_pins[5] = {0, 1, 2, 3, 4};

uint8_t _numlines, _currline;

void MCP_begin(void)
{
  txI2Csize = 2;
  txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
  txI2Cbuf[0] = MCP23017_IODIRA;
  txI2Cbuf[1] = 0xFF;
  I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);

  txI2Cbuf[0] = MCP23017_IODIRB;
  txI2Cbuf[1] = 0xFF;
  I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);

  free((uint8_t *)txI2Cbuf);
  txI2Csize = 0;
}

void MCP_pinMode(uint8_t p, uint8_t d)
{
  uint8_t iodir;
  uint8_t iodiraddr;

  if (p > 15)
    return;

  if (p < 8)
    iodiraddr = MCP23017_IODIRA;
  else
  {
    iodiraddr = MCP23017_IODIRB;
    p -= 8;
  }

  txI2Csize = 1;
  txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
  txI2Cbuf[0] = iodiraddr;
  I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);

  I2C_Read(1, MCP23017_ADDRESS);
  iodir = rxI2Cbuf[0];
  free((uint8_t *)txI2Cbuf);
  txI2Csize = 0;
  printf("PinMode Received: 0x%x\n", iodir);
  free((uint8_t *)rxI2Cbuf);

  if (d == INPUT)
  {
    iodir |= 1 << p;
  }
  else
  {
    iodir &= ~(1 << p);
  }

  txI2Csize = 2;
  txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
  txI2Cbuf[0] = iodiraddr;
  txI2Cbuf[1] = iodir;
  I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
  free((uint8_t *)txI2Cbuf);
  txI2Csize = 0;
}

void MCP_pullUp(uint8_t p, uint8_t d)
{
   uint8_t gppu;
   uint8_t gppuaddr;

   // only 16 bits!
   if (p > 15)
      return;

   if (p < 8)
      gppuaddr = MCP23017_GPPUA;
   else
   {
      gppuaddr = MCP23017_GPPUB;
      p -= 8;
   }

   txI2Csize = 1;
   txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
   txI2Cbuf[0] = gppuaddr;
   I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
   free((uint8_t *)txI2Cbuf);
   txI2Csize = 0;

   I2C_Read(1, MCP23017_ADDRESS);
   gppu = rxI2Cbuf[0];
   free((uint8_t *)rxI2Cbuf);

   // set the pin and direction
   if (d == HIGH)
   {
      gppu |= 1 << p;
   }
   else
   {
      gppu &= ~(1 << p);
   }

   // write the new GPIO
   txI2Csize = 2;
   txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
   txI2Cbuf[0] = gppuaddr;
   txI2Cbuf[1] = gppu;
   I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
   free((uint8_t *)txI2Cbuf);
   txI2Csize = 0;
}

void MCP_digitalWrite(uint8_t p, uint8_t d)
{
   uint8_t gpio;
   uint8_t gpioaddr, olataddr;

   if (p > 15)
       return;

   if (p < 8)
   {
      olataddr = MCP23017_OLATA;
      gpioaddr = MCP23017_GPIOA;
   }
   else
   {
      olataddr = MCP23017_OLATB;
      gpioaddr = MCP23017_GPIOB;
      p -= 8;
   }

   txI2Csize = 1;
   txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
   txI2Cbuf[0] = olataddr;
   I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
   free((uint8_t *)txI2Cbuf);
   txI2Csize = 0;

   I2C_Read(1, MCP23017_ADDRESS);

   gpio = rxI2Cbuf[0];
   free((uint8_t *)rxI2Cbuf);

   if (d == HIGH)
   {
      gpio |= 1 << p;
   }
   else
   {
      gpio &= ~(1 << p);
   }

   txI2Csize = 2;
   txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
   txI2Cbuf[0] = gpioaddr;
   txI2Cbuf[1] = gpio;
   I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
   free((uint8_t *)txI2Cbuf);
   txI2Csize = 0;
}

uint16_t MCP_readGPIOAB(void)
{
   uint16_t ba = 0;

   txI2Csize = 1;
   txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
   txI2Cbuf[0] = MCP23017_GPIOA;
   I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
   free((uint8_t *)txI2Cbuf);
   txI2Csize = 0;

   I2C_Read(2, MCP23017_ADDRESS);
   ba = rxI2Cbuf[1];
   ba = ba << 8;
   ba = ba | rxI2Cbuf[0];
   free((uint8_t *)rxI2Cbuf);

   return ba;
}

void MCP_writeGPIOAB(uint16_t ba)
{
   txI2Csize = 3;
   txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
   txI2Cbuf[0] = MCP23017_GPIOA;
   txI2Cbuf[1] = ba & 0xFF;
   txI2Cbuf[2] = (ba >> 8) & 0xFF;
   I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
   free((uint8_t *)txI2Cbuf);
   txI2Csize = 0;
}

void MCP_write4bits(uint8_t value)
{
   uint16_t out = 0;

   out = MCP_readGPIOAB();

   for (int i = 0; i < 4; i++)
   {
     out &= ~(1 << _data_pins[i]);
     out |= ((value >> i) & 0x1) << _data_pins[i];
   }

   out &= ~(1 << _enable_pin);

   MCP_writeGPIOAB(out);

   MCP_delay_us(1);
   out |= (1 << _enable_pin);
   MCP_writeGPIOAB(out);
   MCP_delay_us(1);
   out &= ~(1 << _enable_pin);
   MCP_writeGPIOAB(out);
   MCP_delay_us(100);
}

void MCP_send(uint8_t value, uint8_t mode)
{
   MCP_digitalWrite(_rs_pin, mode);
   if (_rw_pin != 255)
   {
      MCP_digitalWrite(_rw_pin, LOW);
   }
   MCP_write4bits(value>>4);
   MCP_write4bits(value);
}

void MCP_delay_us(uint16_t us)
{
   uint16_t i, j;

   for (i = 0; i < us; i++)
   {
      for (j = 0; j < DELAY_TICK; j++);
   }
}

void LCD_init(uint8_t lines, uint8_t dotsize)
{
   MCP_begin();
   MCP_pinMode(8, OUTPUT);
   MCP_pinMode(6, OUTPUT);
   MCP_pinMode(7, OUTPUT);
   LCD_setBacklight(0);
   if (_rw_pin)
      MCP_pinMode(_rw_pin, OUTPUT);

   MCP_pinMode(_rs_pin, OUTPUT);
   MCP_pinMode(_enable_pin, OUTPUT);

   for (uint8_t i=0; i<4; i++)
      MCP_pinMode(_data_pins[i], OUTPUT);

   for (uint8_t i=0; i<5; i++)
   {
      MCP_pinMode(_button_pins[i], INPUT);
      MCP_pullUp(_button_pins[i], 1);
   }

   if (lines > 1)
   {
      _displayfunction |= LCD_2LINE;
   }

   _numlines = lines;
   _currline = 0;

   printf("Yahoo\n");

    // for some 1 line displays you can select a 10 pixel high font
   if ((dotsize != 0) && (lines == 1))
   {
      _displayfunction |= LCD_5x10DOTS;
   }
   printf("Nikko\n");
   // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
   // according to datasheet, we need at least 40ms after power rises above 2.7V
   // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
   timer_sleep(50);
   printf("Nikko\n");
   // Now we pull both RS and R/W low to begin commands
   MCP_digitalWrite(_rs_pin, LOW);
   MCP_digitalWrite(_enable_pin, LOW);
   if (_rw_pin != 255)
   {
      MCP_digitalWrite(_rw_pin, LOW);
   }
   //put the LCD into 4 bit or 8 bit mode
   if (! (_displayfunction & LCD_8BITMODE))
   {
      // this is according to the hitachi HD44780 datasheet
      // figure 24, pg 46

      // we start in 8bit mode, try to set 4 bit mode
      MCP_write4bits(0x03);
      timer_sleep(5); // wait min 4.1ms

      // second try
      MCP_write4bits(0x03);
      timer_sleep(5); // wait min 4.1ms

      // third go!
      MCP_write4bits(0x03);
      timer_sleep(1);

      // finally, set to 8-bit interface
      MCP_write4bits(0x02);
   }
   else
   {
      // this is according to the hitachi HD44780 datasheet
      // page 45 figure 23

      // Send function set command sequence
      LCD_command(LCD_FUNCTIONSET | _displayfunction);
      timer_sleep(5);

      // second try
      LCD_command(LCD_FUNCTIONSET | _displayfunction);
      MCP_delay_us(150);

      // third go
      LCD_command(LCD_FUNCTIONSET | _displayfunction);
   }

    // finally, set # lines, font size, etc.
    LCD_command(LCD_FUNCTIONSET | _displayfunction);

    // turn the display on with no cursor or blinking default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    LCD_display();

    // clear it off
    LCD_clear();

    // Initialize to default text direction (for romance languages)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    LCD_command(LCD_ENTRYMODESET | _displaymode);
}

void LCD_setBacklight(uint8_t status)
{
   MCP_digitalWrite(6, status & 0x1);
}

void LCD_command(uint8_t value)
{
   MCP_send(value, LOW);
}

void LCD_display(void)
{
   _displaycontrol |= LCD_DISPLAYON;
   LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD_clear(void)
{
   LCD_command(LCD_CLEARDISPLAY);
   timer_sleep(4);
}

void LCD_write(uint8_t value) {
   MCP_send(value, HIGH);
}

void LCD_setCursor(uint8_t col, uint8_t row)
{
   int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
   if ( row > _numlines ) {
      row = _numlines - 1;
   }
   LCD_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void MCP_print_iocon(void)
{
   uint16_t ba = 0;

   txI2Csize = 1;
   txI2Cbuf = (uint8_t *)malloc(txI2Csize * sizeof(uint8_t));
   txI2Cbuf[0] = MCP23017_IOCONB;
   I2C_Write(txI2Cbuf, txI2Csize, MCP23017_ADDRESS);
   free((uint8_t *)txI2Cbuf);
   txI2Csize = 0;

   I2C_Read(1, MCP23017_ADDRESS);
   ba = rxI2Cbuf[0];
   free((uint8_t *)rxI2Cbuf);
   printf("IOCON: 0x%x\n", ba);
}
