/*!
 *****************************************************************************
 * @file:    Lcd.h
 * @brief:   ST7565R LCD control
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2016-2017 Analog Devices, Inc.

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

#ifndef LCD_H_
#define LCD_H_


/********************************* Definitions ********************************/

#define LCD_COLUMNS     128u
#define LCD_PAGES       4u
#define LCD_LINES       64u

#define UP_X            112
#define LEFT_X          104
#define RIGHT_X         120
#define DOWN_X          112
#define CENTER_X        112

#define ACC_LIMIT       50

/* Font size for Y */
#define FONT_Y_SIZE  8

/* ASCII offset */
#define OFFS_ASCII   32

/* ST7565R LCD  commands */
#define CMD_DISPLAY_OFF          0xAE
#define CMD_DISPLAY_ON           0xAF
#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE             0xB0
#define CMD_SET_COLUMN_UPPER     0x10
#define CMD_SET_COLUMN_LOWER     0x00
#define CMD_SET_ADC_NORMAL       0xA0
#define CMD_SET_ADC_REVERSE      0xA1
#define CMD_SET_DISP_NORMAL      0xA6
#define CMD_SET_DISP_REVERSE     0xA7
#define CMD_SET_ALLPTS_NORMAL    0xA4
#define CMD_SET_ALLPTS_ON        0xA5
#define CMD_SET_BIAS_9           0xA2
#define CMD_SET_BIAS_7           0xA3
#define CMD_RMW                  0xE0
#define CMD_RMW_CLEAR            0xEE
#define CMD_INTERNAL_RESET       0xE2
#define CMD_SET_COM_NORMAL       0xC0
#define CMD_SET_COM_REVERSE      0xC8
#define CMD_SET_POWER_CONTROL    0x28
#define CMD_SET_RESISTOR_RATIO   0x20
#define CMD_SET_VOLUME_FIRST     0x81
#define CMD_SET_VOLUME_SECOND    0
#define CMD_SET_STATIC_OFF       0xAC
#define CMD_SET_STATIC_ON        0xAD
#define CMD_SET_STATIC_REG       0x0
#define CMD_SET_BOOSTER_FIRST    0xF8
#define CMD_SET_BOOSTER_234      0
#define CMD_SET_BOOSTER_5        1
#define CMD_SET_BOOSTER_6        3
#define CMD_NOP                  0xE3
#define CMD_TEST                 0xF0


/****************************** Global Data ***********************************/

extern const uint8_t pui8Rec8x8[8];
extern const uint8_t pui8RecInv8x8[8];

/************************* Global functions prototypes*************************/

void Lcd_Init(void);
void Lcd_DisplayString(uint8_t ui8row, uint8_t ui8col, int8_t *pi8str);
void Lcd_DisplaySymbol(uint8_t ui8row, uint8_t ui8col, uint8_t ui8width, const uint8_t *pui8symbol);
void Lcd_FillPages(uint8_t ui8start, uint8_t ui8num, uint8_t ui8Data);
void Lcd_SetLine(uint8_t ui8line);
void Lcd_Run();


#endif /* LCD_H_ */

/* End Of File */

