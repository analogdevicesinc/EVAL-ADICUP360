/*!
 *****************************************************************************
 * @file:    lcd.h
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _LCD_H_
   #define _LCD_H_

   #define MCP23017_ADDRESS   0x20

   #define MCP23017_IODIRA    0x00
   #define MCP23017_IPOLA     0x02
   #define MCP23017_GPINTENA  0x04
   #define MCP23017_DEFVALA   0x06
   #define MCP23017_INTCONA   0x08
   #define MCP23017_IOCONA    0x0A
   #define MCP23017_GPPUA     0x0C
   #define MCP23017_INTFA     0x0E
   #define MCP23017_INTCAPA   0x10
   #define MCP23017_GPIOA     0x12
   #define MCP23017_OLATA     0x14

   #define MCP23017_IODIRB    0x01
   #define MCP23017_IPOLB     0x03
   #define MCP23017_GPINTENB  0x05
   #define MCP23017_DEFVALB   0x07
   #define MCP23017_INTCONB   0x09
   #define MCP23017_IOCONB    0x0B
   #define MCP23017_GPPUB     0x0D
   #define MCP23017_INTFB     0x0F
   #define MCP23017_INTCAPB   0x11
   #define MCP23017_GPIOB     0x13
   #define MCP23017_OLATB     0x15

   // commands
   #define LCD_CLEARDISPLAY 0x01
   #define LCD_RETURNHOME 0x02
   #define LCD_ENTRYMODESET 0x04
   #define LCD_DISPLAYCONTROL 0x08
   #define LCD_CURSORSHIFT 0x10
   #define LCD_FUNCTIONSET 0x20
   #define LCD_SETCGRAMADDR 0x40
   #define LCD_SETDDRAMADDR 0x80

   // flags for display entry mode
   #define LCD_ENTRYRIGHT 0x00
   #define LCD_ENTRYLEFT 0x02
   #define LCD_ENTRYSHIFTINCREMENT 0x01
   #define LCD_ENTRYSHIFTDECREMENT 0x00

   // flags for display on/off control
   #define LCD_DISPLAYON 0x04
   #define LCD_DISPLAYOFF 0x00
   #define LCD_CURSORON 0x02
   #define LCD_CURSOROFF 0x00
   #define LCD_BLINKON 0x01
   #define LCD_BLINKOFF 0x00

   // flags for display/cursor shift
   #define LCD_DISPLAYMOVE 0x08
   #define LCD_CURSORMOVE 0x00
   #define LCD_MOVERIGHT 0x04
   #define LCD_MOVELEFT 0x00

   // flags for function set
   #define LCD_8BITMODE 0x10
   #define LCD_4BITMODE 0x00
   #define LCD_2LINE 0x08
   #define LCD_1LINE 0x00
   #define LCD_5x10DOTS 0x04
   #define LCD_5x8DOTS 0x00

   #define BUTTON_UP 0x08
   #define BUTTON_DOWN 0x04
   #define BUTTON_LEFT 0x10
   #define BUTTON_RIGHT 0x02
   #define BUTTON_SELECT 0x01

   #define INPUT  1
   #define OUTPUT 0

   #define HIGH   1
   #define LOW    0

   #define DELAY_TICK 30

   void MCP_begin (void);
   void MCP_pinMode(uint8_t p, uint8_t d);
   void MCP_pullUp(uint8_t p, uint8_t d);
   void MCP_digitalWrite(uint8_t p, uint8_t d);
   uint16_t MCP_readGPIOAB(void);
   void MCP_writeGPIOAB(uint16_t ba);
   void MCP_write4bits(uint8_t value);
   void MCP_send(uint8_t value, uint8_t mode);
   void MCP_delay_us(uint16_t us);
   void MCP_print_iocon(void);
   void LCD_init(uint8_t lines, uint8_t dotsize);
   void LCD_setBacklight(uint8_t status);
   void LCD_command(uint8_t value);
   void LCD_display(void);
   void LCD_clear(void);
   void LCD_write(uint8_t value);
   void LCD_setCursor(uint8_t col, uint8_t row);

#endif
