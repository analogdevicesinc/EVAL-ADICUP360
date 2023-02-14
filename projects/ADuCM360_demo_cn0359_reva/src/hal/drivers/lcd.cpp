/***************************************************************************//**
 *   @file   lcd.cpp
 *   @brief  Implementation of lcd.cpp
 *   @author
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <ADuCM360.h>
#include <hal/timer.h>
#include <errno.h>
#include <cstring>
#include <hal/drivers/dma.h>
#include <hal/drivers/lcd.h>
#include <cassert>
#include <hal/drivers/flash.h>
#include <hal/devices.h>

#define LCD_REVERSE 0x1
#define LCD_BLINK 0x2

#define LCD_BLINK_INTERVAL 500

#define FB_FLUSH_DELAY 30

uint32_t framebuffer_memory[8][132 / 4]; //1056 bytes

static unsigned char lcd_tty_buffer[16 * 4 * 2];

static int tty_cursor = 0;

static bool blink_on = false;

static const uint32_t ASC16[256][4] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x849404f8, 0xf8049484,
                                       0x09080807, 0x07080809, 0x7c6cfcf8, 0xf8fc6c7c, 0x0e0f0f07, 0x070f0f0e,
                                       0xe0f0f0e0, 0x00e0f0f0, 0x0f070301, 0x00010307, 0xf0e0c080, 0x0080c0e0,
                                       0x07030100, 0x00000103, 0x38f0c0c0, 0xc0c0f038, 0x0e090101, 0x0101090e,
                                       0xf8f0e0c0, 0xc0e0f0f8, 0x0f090100, 0x0001090f, 0xc0800000, 0x000080c0,
                                       0x03010000, 0x00000103, 0x3f7fffff, 0xffff7f3f, 0xfcfeffff, 0xfffffefc,
                                       0x2060c000, 0x00c06020, 0x04060300, 0x00030604, 0xdf9f3fff, 0xff3f9fdf,
                                       0xfbf9fcff, 0xfffcf9fb, 0x7460c080, 0x003c8cdc, 0x08080f07, 0x0000070f,
                                       0x84fc7800, 0x0078fc84, 0x0f020200, 0x0002020f, 0xfcfc0000, 0x1c1c1414,
                                       0x070f0e0c, 0x00000000, 0x14fcfc00, 0xfcfc1414, 0x000f1f1c, 0x070f0e00,
                                       0x78c0a0a0, 0xa0a0c078, 0x0f010202, 0x0202010f, 0xf0f8fcfe, 0x004040e0,
                                       0x0103070f, 0x00000000, 0xf0e04040, 0x00fefcf8, 0x01000000, 0x000f0703,
                                       0xfc181000, 0x001018fc, 0x07030100, 0x00010307, 0x00fcfc00, 0x00fcfc00,
                                       0x000d0d00, 0x000d0d00, 0xfc447c38, 0xfcfc04fc, 0x0f000000, 0x0f0f000f,
                                       0x123aeec4, 0x00c4e632, 0x12131908, 0x00081d17, 0x00000000, 0x00000000,
                                       0x0f0f0f0f, 0x000f0f0f, 0xfc181000, 0x001018fc, 0x0f0b0900, 0x00090b0f,
                                       0xfc181000, 0x001018fc, 0x0f000000, 0x0000000f, 0xfc000000, 0x000000fc,
                                       0x0f060200, 0x0002060f, 0xa0808080, 0x0080c0e0, 0x02000000, 0x00000103,
                                       0xa0e0c080, 0x00808080, 0x02030100, 0x00000000, 0x0000c0c0, 0x00000000,
                                       0x02020303, 0x00020202, 0x80e0c080, 0x0080c0e0, 0x00030100, 0x00000103,
                                       0xf0e08000, 0x000080e0, 0x07070706, 0x00060707, 0xf0f0f030, 0x0030f0f0,
                                       0x07030000, 0x00000003, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                       0xfc380000, 0x000038fc, 0x0d000000, 0x0000000d, 0x001e0e00, 0x000e1e00,
                                       0x00000000, 0x00000000, 0x20f8f820, 0x0020f8f8, 0x020f0f02, 0x00020f0f,
                                       0x47447c38, 0x0098cc47, 0x38080c06, 0x00070f38, 0x80003030, 0x003060c0,
                                       0x0103060c, 0x000c0c00, 0xe47cd880, 0x0040d8bc, 0x08080f07, 0x00080f07,
                                       0x0e1e1000, 0x00000000, 0x00000000, 0x00000000, 0xf8f00000, 0x0000040c,
                                       0x07030000, 0x0000080c, 0x0c040000, 0x0000f0f8, 0x0c080000, 0x00000307,
                                       0xc0e0a080, 0x80a0e0c0, 0x01030200, 0x00020301, 0xe0808000, 0x008080e0,
                                       0x03000000, 0x00000003, 0x00000000, 0x00000000, 0x1e100000, 0x0000000e,
                                       0x80808080, 0x00808080, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                       0x0c000000, 0x0000000c, 0x80000000, 0x003060c0, 0x0103060c, 0x00000000,
                                       0xc40cf8f0, 0x00f0f80c, 0x080c0703, 0x0003070c, 0xfc181000, 0x000000fc,
                                       0x0f080800, 0x0008080f, 0xc4840c08, 0x00183c64, 0x08090f0e, 0x000c0c08,
                                       0x44440c08, 0x00b8fc44, 0x08080c04, 0x00070f08, 0x98b0e0c0, 0x0080fcfc,
                                       0x08000000, 0x00080f0f, 0x44447c7c, 0x0084c444, 0x08080c04, 0x00070f08,
                                       0x444cf8f0, 0x0080c044, 0x08080f07, 0x00070f08, 0x84040c0c, 0x003c7cc4,
                                       0x0f0f0000, 0x00000000, 0x4444fcb8, 0x00b8fc44, 0x08080f07, 0x00070f08,
                                       0x44447c38, 0x00f8fc44, 0x08080800, 0x0003070c, 0x30000000, 0x00000030,
                                       0x06000000, 0x00000006, 0x30000000, 0x00000030, 0x0e080000, 0x00000006,
                                       0x60c08000, 0x00081830, 0x03010000, 0x00080c06, 0x20202000, 0x00202020,
                                       0x01010100, 0x00010101, 0x30180800, 0x0080c060, 0x060c0800, 0x00000103,
                                       0xc4041c18, 0x00183ce4, 0x0d000000, 0x0000000d, 0xc808f8f0, 0x00f0f8c8,
                                       0x0b080f07, 0x00010b0b, 0x8c98f0e0, 0x00e0f098, 0x00000f0f, 0x000f0f00,
                                       0x44fcfc04, 0x00b8fc44, 0x080f0f08, 0x00070f08, 0x040cf8f0, 0x00180c04,
                                       0x080c0703, 0x00060c08, 0x04fcfc04, 0x00f0f80c, 0x080f0f08, 0x0003070c,
                                       0x44fcfc04, 0x001c0ce4, 0x080f0f08, 0x000e0c08, 0x44fcfc04, 0x001c0ce4,
                                       0x080f0f08, 0x00000000, 0x840cf8f0, 0x00988c84, 0x080c0703, 0x000f0708,
                                       0x4040fcfc, 0x00fcfc40, 0x00000f0f, 0x000f0f00, 0xfc040000, 0x000004fc,
                                       0x0f080000, 0x0000080f, 0x04000000, 0x0004fcfc, 0x08080f07, 0x0000070f,
                                       0xc0fcfc04, 0x001c3ce0, 0x000f0f08, 0x000e0f01, 0x04fcfc04, 0x00000000,
                                       0x080f0f08, 0x000e0c08, 0x7038fcfc, 0x00fcfc38, 0x00000f0f, 0x000f0f00,
                                       0x7038fcfc, 0x00fcfce0, 0x00000f0f, 0x000f0f00, 0x0404fcf8, 0x00f8fc04,
                                       0x08080f07, 0x00070f08, 0x44fcfc04, 0x00387c44, 0x080f0f08, 0x00000000,
                                       0x0404fcf8, 0x00f8fc04, 0x0e080f07, 0x00273f3c, 0x44fcfc04, 0x0038fcc4,
                                       0x000f0f08, 0x000f0f00, 0x44643c18, 0x00189cc4, 0x08080e06, 0x00070f08,
                                       0xfc0c1c00, 0x001c0cfc, 0x0f080000, 0x0000080f, 0x0000fcfc, 0x00fcfc00,
                                       0x08080f07, 0x00070f08, 0x0000fcfc, 0x00fcfc00, 0x0c060301, 0x00010306,
                                       0xc000fcfc, 0x00fcfc00, 0x030e0f07, 0x00070f0e, 0xe0f03c0c, 0x000c3cf0,
                                       0x01030f0c, 0x000c0f03, 0xc07c3c00, 0x003c7cc0, 0x0f080000, 0x0000080f,
                                       0xc4840c1c, 0x001c3c64, 0x08090f0e, 0x000e0c08, 0xfcfc0000, 0x00000404,
                                       0x0f0f0000, 0x00000808, 0xc0e07038, 0x00000080, 0x01000000, 0x000e0703,
                                       0x04040000, 0x0000fcfc, 0x08080000, 0x00000f0f, 0x03060c08, 0x00080c06,
                                       0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x20202020, 0x20202020,
                                       0x07030000, 0x00000004, 0x00000000, 0x00000000, 0xa0a0a000, 0x0000c0e0,
                                       0x08080f07, 0x00080f07, 0x20fcfc04, 0x0080c060, 0x080f0f00, 0x00070f08,
                                       0x2020e0c0, 0x00406020, 0x08080f07, 0x00040c08, 0x2460c080, 0x0000fcfc,
                                       0x08080f07, 0x00080f07, 0xa0a0e0c0, 0x00c0e0a0, 0x08080f07, 0x00040c08,
                                       0x44fcf840, 0x0000180c, 0x080f0f08, 0x00000000, 0x2020e0c0, 0x0020e0c0,
                                       0x48486f27, 0x00003f7f, 0x40fcfc04, 0x00c0e020, 0x000f0f08, 0x000f0f00,
                                       0xec200000, 0x000000ec, 0x0f080000, 0x0000080f, 0x00000000, 0x00ecec20,
                                       0x40703000, 0x003f7f40, 0x80fcfc04, 0x002060c0, 0x010f0f08, 0x000c0e03,
                                       0xfc040000, 0x000000fc, 0x0f080000, 0x0000080f, 0xc060e0e0, 0x00c0e060,
                                       0x07000f0f, 0x000f0f00, 0x20c0e020, 0x00c0e020, 0x000f0f00, 0x000f0f00,
                                       0x2020e0c0, 0x00c0e020, 0x08080f07, 0x00070f08, 0x20c0e020, 0x00c0e020,
                                       0x487f7f40, 0x00070f08, 0x2020e0c0, 0x0020e0c0, 0x48080f07, 0x00407f7f,
                                       0x60c0e020, 0x00c0e020, 0x080f0f08, 0x00000000, 0x20a0e040, 0x00406020,
                                       0x09090c04, 0x00040e0b, 0xfcf82020, 0x00002020, 0x0f070000, 0x00040c08,
                                       0x0000e0e0, 0x0000e0e0, 0x08080f07, 0x00080f07, 0x00e0e000, 0x00e0e000,
                                       0x0c070300, 0x0003070c, 0x8000e0e0, 0x00e0e000, 0x070c0f07, 0x00070f0c,
                                       0x80c06020, 0x002060c0, 0x03070c08, 0x00080c07, 0x0000e0e0, 0x00e0e000,
                                       0x48484f47, 0x001f3f68, 0xa0206060, 0x002060e0, 0x090b0e0c, 0x000c0c08,
                                       0xf8404000, 0x000404bc, 0x07000000, 0x0008080f, 0xbc000000, 0x000000bc,
                                       0x0f000000, 0x0000000f, 0xbc040400, 0x004040f8, 0x0f080800, 0x00000007,
                                       0x0c040c08, 0x00040c08, 0x00000000, 0x00000000, 0x3060c080, 0x0080c060,
                                       0x04040707, 0x00070704, 0x040cf8f0, 0x00180c04, 0x24262301, 0x00133e2c,
                                       0x0000e4e4, 0x0000e4e4, 0x08080f07, 0x00080f07, 0xaca8e0c0, 0x00c0e2a6,
                                       0x08080f07, 0x00040c08, 0xa6aca800, 0x0000c8ec, 0x08080f07, 0x00080f07,
                                       0xa0a0a404, 0x0000c4e4, 0x08080f07, 0x00080f07, 0xaca6a200, 0x0000c0e8,
                                       0x08080f07, 0x00080f07, 0xaaaea400, 0x0000c4ee, 0x08080f07, 0x00080f07,
                                       0x10f0e000, 0x00203010, 0x12130100, 0x00091f16, 0xa6ace8c0, 0x00c0e8ac,
                                       0x08080f07, 0x00040c08, 0xa0a0e4c4, 0x00c4e4a0, 0x08080f07, 0x00040c08,
                                       0xaca6e2c0, 0x00c0e0a8, 0x08080f07, 0x00040c08, 0xe0240400, 0x000404e0,
                                       0x0f080000, 0x0000080f, 0xe62c0800, 0x00080ce6, 0x0f080000, 0x0000080f,
                                       0xec260200, 0x000000e8, 0x0f080000, 0x0000080f, 0x1830e2c2, 0x00c2e230,
                                       0x01010f0f, 0x000f0f01, 0x1537e2c0, 0x00c0e237, 0x01010f0f, 0x000f0f01,
                                       0x93f6f410, 0x0030b091, 0x080f0f08, 0x000c0c08, 0xc0c06020, 0x00c0e020,
                                       0x07090f06, 0x0009090f, 0x444cf8f0, 0x0044fcfc, 0x00000f0f, 0x00080f0f,
                                       0x262ce8c0, 0x00c0e82c, 0x08080f07, 0x00070f08, 0x2020e4c4, 0x00c4e420,
                                       0x08080f07, 0x00070f08, 0x2c26e2c0, 0x00c0e028, 0x08080f07, 0x00070f08,
                                       0x0606ece8, 0x0000e8ec, 0x08080f07, 0x00080f07, 0x0c06e2e0, 0x0000e0e8,
                                       0x08080f07, 0x00080f07, 0x0000e4e4, 0x00e4e400, 0x48484f07, 0x001f3f68,
                                       0x0808faf2, 0x00f2fa08, 0x08080f07, 0x00070f08, 0x0000fafa, 0x00fafa00,
                                       0x08080f07, 0x00070f08, 0x0ef8f000, 0x0010180e, 0x0e030100, 0x0001030e,
                                       0x22fefc20, 0x00000c06, 0x080f0f0c, 0x00040c08, 0xf05c4c00, 0x004c5cf0,
                                       0x0f010100, 0x0001010f, 0x9212fefe, 0x0080ecde, 0x00000f0f, 0x00080f07,
                                       0xfc404000, 0x044642fe, 0x3f203010, 0x0000001f, 0xa6aca800, 0x0000c0e2,
                                       0x08080f07, 0x00080f07, 0xec280000, 0x000002e6, 0x0f080000, 0x0000080f,
                                       0x262ce8c0, 0x00c0e022, 0x08080f07, 0x00070f08, 0x060ce8e0, 0x0000e0e2,
                                       0x08080f07, 0x00080f07, 0x2cc4ec28, 0x00c4ec28, 0x000f0f00, 0x000f0f00,
                                       0xe371fbfa, 0x00f9fbc2, 0x00000f0f, 0x000f0f01, 0x525e4c00, 0x00505e5e,
                                       0x00000000, 0x00000000, 0x525e4c00, 0x00004c5e, 0x00000000, 0x00000000,
                                       0x6cec8000, 0x00000000, 0x08080f07, 0x00060e08, 0x4040c0c0, 0x00404040,
                                       0x00000707, 0x00000000, 0x40404040, 0x00c0c040, 0x00000000, 0x00070700,
                                       0xc0803e3e, 0x00183060, 0x32210306, 0x00242e3a, 0xc0803e3e, 0x00183060,
                                       0x0c090306, 0x003f3f0e, 0xec000000, 0x000000ec, 0x0f070000, 0x0000070f,
                                       0xa060c080, 0x002060c0, 0x02030100, 0x00020301, 0xa0c06020, 0x0080c060,
                                       0x02010302, 0x00000103, 0x5500aa00, 0x5500aa00, 0x5500aa00, 0x5500aa00,
                                       0x55aa55aa, 0x55aa55aa, 0x55aa55aa, 0x55aa55aa, 0xffaaff55, 0xffaaff55,
                                       0xffaaff55, 0xffaaff55, 0xff000000, 0x000000ff, 0xff000000, 0x000000ff,
                                       0xff808080, 0x000000ff, 0xff000000, 0x000000ff, 0xffa0a0a0, 0x000000ff,
                                       0xff000000, 0x000000ff, 0xffff8080, 0x00ffff00, 0xffff0000, 0x00ffff00,
                                       0x80808080, 0x00808080, 0xffff0000, 0x00ffff00, 0xe0a0a0a0, 0x000000e0,
                                       0xff000000, 0x000000ff, 0xbfbfa0a0, 0x00ffff00, 0xffff0000, 0x00ffff00,
                                       0xffff0000, 0x00ffff00, 0xffff0000, 0x00ffff00, 0xa0a0a0a0, 0x00e0e020,
                                       0xffff0000, 0x00ffff00, 0xbfbfa0a0, 0x00ffff80, 0x00000000, 0x00000000,
                                       0xffff8080, 0x00ffff80, 0x00000000, 0x00000000, 0xffa0a0a0, 0x000000ff,
                                       0x00000000, 0x00000000, 0x80808080, 0x00000080, 0xff000000, 0x000000ff,
                                       0xff000000, 0x808080ff, 0x00000000, 0x00000000, 0xff808080, 0x808080ff,
                                       0x00000000, 0x00000000, 0x80808080, 0x80808080, 0xff000000, 0x000000ff,
                                       0xff000000, 0x808080ff, 0xff000000, 0x000000ff, 0x80808080, 0x80808080,
                                       0x00000000, 0x00000000, 0xff808080, 0x808080ff, 0xff000000, 0x000000ff,
                                       0xff000000, 0xa0a0a0ff, 0xff000000, 0x000000ff, 0xffff0000, 0x80ffff00,
                                       0xffff0000, 0x00ffff00, 0xffff0000, 0xa0bfbf80, 0x00000000, 0x00000000,
                                       0xe0e00000, 0xa0a0a020, 0xffff0000, 0x00ffff00, 0xbfbfa0a0, 0xa0bfbf80,
                                       0x00000000, 0x00000000, 0xa0a0a0a0, 0xa0a0a020, 0xffff0000, 0x00ffff00,
                                       0xffff0000, 0xa0bfbf00, 0xffff0000, 0x00ffff00, 0xa0a0a0a0, 0xa0a0a0a0,
                                       0x00000000, 0x00000000, 0xbfbfa0a0, 0xa0bfbf00, 0xffff0000, 0x00ffff00,
                                       0xbfa0a0a0, 0xa0a0a0bf, 0x00000000, 0x00000000, 0xffff8080, 0x80ffff80,
                                       0x00000000, 0x00000000, 0xa0a0a0a0, 0xa0a0a0a0, 0xff000000, 0x000000ff,
                                       0x80808080, 0x80808080, 0xffff0000, 0x00ffff00, 0xffff0000, 0x80ffff80,
                                       0x00000000, 0x00000000, 0xff000000, 0xa0a0a0ff, 0x00000000, 0x00000000,
                                       0xe0000000, 0xa0a0a0e0, 0xff000000, 0x000000ff, 0x80800000, 0x80808080,
                                       0xffff0000, 0x00ffff00, 0xffff8080, 0x80ffff80, 0xffff0000, 0x00ffff00,
                                       0xffa0a0a0, 0xa0a0a0ff, 0xff000000, 0x000000ff, 0xff808080, 0x000000ff,
                                       0x00000000, 0x00000000, 0x80000000, 0x80808080, 0xff000000, 0x000000ff,
                                       0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x80808080, 0x80808080,
                                       0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff, 0x00000000,
                                       0x00000000, 0xffffffff, 0x00000000, 0xffffffff, 0x7f7f7f7f, 0x7f7f7f7f,
                                       0x00000000, 0x00000000, 0xe020e0c0, 0x002060c0, 0x0f080f07, 0x00080c07,
                                       0x4404fcf8, 0x0000b8fc, 0x00000f0f, 0x00070f08, 0x0404fcfc, 0x001c1c04,
                                       0x00000f0f, 0x00000000, 0x10f0f010, 0x0010f0f0, 0x000f0f00, 0x00000f0f,
                                       0xc8683818, 0x00181888, 0x090b0e0c, 0x000c0c08, 0xe020e0c0, 0x002020e0,
                                       0x0f080f07, 0x00000007, 0x00f0f000, 0x00f0f000, 0x020f1f10, 0x00010302,
                                       0xf0103020, 0x001030e0, 0x0f000000, 0x0000000f, 0x38e8c800, 0x00c8e838,
                                       0x0e0b0900, 0x00090b0e, 0x8898f0e0, 0x00e0f098, 0x080c0703, 0x0003070c,
                                       0x048cf870, 0x0070f88c, 0x000f0f08, 0x00080f0f, 0x5cc88000, 0x00c4e474,
                                       0x080f0700, 0x00070f08, 0xe020e0c0, 0xc0e020e0, 0x03020301, 0x01030203,
                                       0xe020e0c0, 0xc8f830e0, 0x03070f09, 0x01030202, 0x4cf8f000, 0x00004444,
                                       0x0c070300, 0x00000808, 0x0808f8f0, 0x00f0f808, 0x00000f0f, 0x000f0f00,
                                       0x90909090, 0x00909090, 0x04040404, 0x00040404, 0xf0404000, 0x004040f0,
                                       0x09080808, 0x08080809, 0x18080000, 0x0040e0b0, 0x0b0a0800, 0x00080809,
                                       0xb0e04000, 0x00000818, 0x09080800, 0x00080a0b, 0xf8000000, 0x181c04fc,
                                       0xff000000, 0x000000ff, 0xff000000, 0x000000ff, 0x0f080f07, 0x00000007,
                                       0xb0808000, 0x008080b0, 0x06000000, 0x00000006, 0x60206040, 0x00206040,
                                       0x03010302, 0x00010302, 0x121e0c00, 0x00000c1e, 0x00000000, 0x00000000,
                                       0x80000000, 0x00000080, 0x01000000, 0x00000001, 0x00000000, 0x00000000,
                                       0x01000000, 0x00000001, 0x00808080, 0x0202fefe, 0x0c070300, 0x00000f0f,
                                       0x027c7e02, 0x00007c7e, 0x00000000, 0x00000000, 0x4e5a7664, 0x00000064,
                                       0x00000000, 0x00000000, 0xf0f0f000, 0x0000f0f0, 0x07070700, 0x00000707,
                                       0x00000000, 0x00000000, 0x00000000, 0x00000000};

void static lcd_write_control(const unsigned char &data)
{
	GP0CLR_CLR4_BBA = true;
	pADI_SPI1->SPITX = data;
	while ((pADI_SPI1->SPISTA & SPI1STA_TXFSTA_MSK) != SPI1STA_TXFSTA_EMPTY)
		;
}

void static lcd_frambuffer_flush(void)
{
	timer t;
	t.time = FB_FLUSH_DELAY;
	t.timer_app.fun = enable_dma_spi1tx;
	t.timer_app.argc = 0;
	t.timer_app.argv = new char*;

	update_timer(t);
}

/*
 * lcd tty buffer size 128
 * address 0~63 for character display
 * address 64~127 for reverse and blink control
 *
 * character register address map to LCD position
 * 8 x 16 pixel for each position
 * ┌───┬───┬──────┬───┬───┐
 * │ 0 │ 1 │......│14 │15 │
 * ├───┼───┼──────┼───┼───┤
 * │16 │17 │......│30 │31 │
 * ├───┼───┼──────┼───┼───┤
 * │32 │33 │......│46 │47 │
 * ├───┼───┼──────┼───┼───┤
 * │48 │49 │......│62 │63 │
 * └───┴───┴──────┴───┴───┘
 *
 * reverse and blink register address map to LCD position
 * 8 x 16 pixel for each position
 * ┌───┬───┬──────┬───┬───┐
 * │64 │65 │......│78 │79 │
 * ├───┼───┼──────┼───┼───┤
 * │80 │81 │......│94 │95 │
 * ├───┼───┼──────┼───┼───┤
 * │96 │97 │......│110│111│
 * ├───┼───┼──────┼───┼───┤
 * │112│113│......│126│127│
 * └───┴───┴──────┴───┴───┘
 *
 * reverse and blink register bit definition
 * B: 1 blink, 0 gaze
 * R: 1 reverse, 0 normal
 * others: null
 * ┌─┬─┬─┬─┬─┬─┬─┬─┐
 * │7│6│5│4│3│2│B│R│
 * └─┴─┴─┴─┴─┴─┴─┴─┘
 */

ssize_t lcd_tty_write(const void *buf, size_t count)
{
	if ((tty_cursor + count) > sizeof(lcd_tty_buffer))
	{
		count = sizeof(lcd_tty_buffer) - tty_cursor;
	}

	memcpy(&lcd_tty_buffer[tty_cursor], buf, count);

	for (int j = 0; j < count; ++j)
	{
		int cusror = (tty_cursor + j) % 64;

		int _2y = (cusror >> 3) & 0xfffffffe;
		int _2x = (cusror & 0xf) << 1;

		if ((((lcd_tty_buffer[cusror + 64] & LCD_BLINK) == LCD_BLINK) && blink_on)
				!= ((lcd_tty_buffer[cusror + 64] & LCD_REVERSE) == LCD_REVERSE))
		{
			framebuffer_memory[_2y][_2x + 1] = ~ASC16[lcd_tty_buffer[cusror]][0];

			framebuffer_memory[_2y][_2x + 2] = ~ASC16[lcd_tty_buffer[cusror]][1];

			framebuffer_memory[_2y + 1][_2x + 1] = ~ASC16[lcd_tty_buffer[cusror]][2];

			framebuffer_memory[_2y + 1][_2x + 2] = ~ASC16[lcd_tty_buffer[cusror]][3];
		}
		else
		{
			framebuffer_memory[_2y][_2x + 1] = ASC16[lcd_tty_buffer[cusror]][0];

			framebuffer_memory[_2y][_2x + 2] = ASC16[lcd_tty_buffer[cusror]][1];

			framebuffer_memory[_2y + 1][_2x + 1] = ASC16[lcd_tty_buffer[cusror]][2];

			framebuffer_memory[_2y + 1][_2x + 2] = ASC16[lcd_tty_buffer[cusror]][3];
		}
	}

	lcd_frambuffer_flush();

	tty_cursor += count;

	return count;
}

int static lcd_blinking(int argc, char *argv[])
{
	blink_on = !blink_on;

	for (int i = 0; i < 64; ++i)
	{
		if ((lcd_tty_buffer[i + 64] & LCD_BLINK) == LCD_BLINK)
		{
			int _2y = (i >> 3) & 0xfffffffe;
			int _2x = (i & 0xf) << 1;

			if (((lcd_tty_buffer[i + 64] & LCD_REVERSE) == LCD_REVERSE) != blink_on)
			{
				framebuffer_memory[_2y][_2x + 1] = ~ASC16[lcd_tty_buffer[i]][0];

				framebuffer_memory[_2y][_2x + 2] = ~ASC16[lcd_tty_buffer[i]][1];

				framebuffer_memory[_2y + 1][_2x + 1] = ~ASC16[lcd_tty_buffer[i]][2];

				framebuffer_memory[_2y + 1][_2x + 2] = ~ASC16[lcd_tty_buffer[i]][3];
			}
			else
			{
				framebuffer_memory[_2y][_2x + 1] = ASC16[lcd_tty_buffer[i]][0];

				framebuffer_memory[_2y][_2x + 2] = ASC16[lcd_tty_buffer[i]][1];

				framebuffer_memory[_2y + 1][_2x + 1] = ASC16[lcd_tty_buffer[i]][2];

				framebuffer_memory[_2y + 1][_2x + 2] = ASC16[lcd_tty_buffer[i]][3];
			}
		}
	}

	lcd_frambuffer_flush();

	timer t;
	t.time = LCD_BLINK_INTERVAL / 2;
	t.timer_app.fun = lcd_blinking;
	t.timer_app.argc = 0;
	t.timer_app.argv = new char*;

	update_timer(t);

	return 0;
}

off_t lcd_tty_lseek(off_t offset, int whence)
{
	switch (whence)
	{
	case SEEK_SET:
		if (offset > sizeof(lcd_tty_buffer) || offset < 0)
		{
			errno = EINVAL;
			return -1;
		}
		else
		{
			tty_cursor = offset;
		}

		break;
	case SEEK_CUR:
		if ((tty_cursor + offset) > sizeof(lcd_tty_buffer) || (tty_cursor + offset) < 0)
		{
			errno = EINVAL;
			return -1;
		}
		else
		{
			tty_cursor += offset;
		}
		break;
	case SEEK_END:
		if (offset > 0 || (sizeof(lcd_tty_buffer) + offset) < 0)
		{
			errno = EINVAL;
			return -1;
		}
		else
		{
			tty_cursor = sizeof(lcd_tty_buffer) + offset;
		}
		break;
	default:
		errno = ENXIO;
		return -1;
		break;
	}

	return tty_cursor;
}

void lcd_open(void)
{
	CLKDIS_DISSPI1CLK_BBA = false;
	pADI_CLKCTL->CLKCON1 = (pADI_CLKCTL->CLKCON1 & ~CLKCON1_SPI1CD_MSK) | CLKCON1_SPI1CD_DIV1;

//	according schematic
	pADI_GP0->GPCON = (pADI_GP0->GPCON & ~GP0CON_CON1_MSK) | GP0CON_CON1_SPI1SCLK;
	pADI_GP0->GPCON = (pADI_GP0->GPCON & ~GP0CON_CON2_MSK) | GP0CON_CON2_SPI1MOSI;
	pADI_GP0->GPCON = (pADI_GP0->GPCON & ~GP0CON_CON3_MSK) | GP0CON_CON3_SPI1CS0;
	pADI_GP0->GPCON = (pADI_GP0->GPCON & ~GP0CON_CON4_MSK) | GP0CON_CON4_GPIO;

	GP0OCE_OCE1_BBA = false;
	GP0OCE_OCE2_BBA = false;
	GP0OCE_OCE3_BBA = false;
	GP0OCE_OCE4_BBA = false;

	GP0OEN_OEN1_BBA = true;
	GP0OEN_OEN2_BBA = true;
	GP0OEN_OEN3_BBA = true;
	GP0OEN_OEN4_BBA = true;

	GP0PUL_PUL1_BBA = true;
	GP0PUL_PUL2_BBA = true;
	GP0PUL_PUL3_BBA = true;
	GP0PUL_PUL4_BBA = true;

	pADI_SPI1->SPIDIV = SPI1DIV_BCRST_DIS | (SPI1DIV_DIV_MSK & 0x0);

	pADI_SPI1->SPICON = SPI1CON_MOD_TX4RX4 | SPI1CON_TFLUSH_DIS | SPI1CON_RFLUSH_EN | SPI1CON_CON_EN
	                    | SPI1CON_LOOPBACK_DIS | SPI1CON_SOEN_DIS | SPI1CON_RXOF_EN | SPI1CON_ZEN_DIS | SPI1CON_TIM_TXWR
	                    | SPI1CON_LSB_DIS | SPI1CON_WOM_DIS | SPI1CON_CPOL_HIGH | SPI1CON_CPHA_SAMPLETRAILING
	                    | SPI1CON_MASEN_EN | SPI1CON_ENABLE_EN;

	pADI_SPI1->SPIDMA = SPI1DMA_IENTXDMA_EN | SPI1DMA_ENABLE_EN;

	initial_dma();

	lcd_write_control(0xe2); //reset

	flash_file * p_flash_file;
	decltype(flash_file::lcd_uc1601s_tc) lcd_uc1601s_tc;
	fseek(p_flash, (int) (&p_flash_file->lcd_uc1601s_tc) - (int) (p_flash_file), SEEK_SET);
	fread(&lcd_uc1601s_tc, sizeof(flash_file::lcd_uc1601s_tc), 1, p_flash);

	lcd_write_control(0x24 | (lcd_uc1601s_tc & 0x3)); //set lcd temperature compensation

	decltype(flash_file::lcd_uc1601s_br) lcd_uc1601s_br;
	fseek(p_flash, (int) (&p_flash_file->lcd_uc1601s_br) - (int) (p_flash_file), SEEK_SET);
	fread(&lcd_uc1601s_br, sizeof(flash_file::lcd_uc1601s_br), 1, p_flash);

	lcd_write_control(0xe8 | (lcd_uc1601s_br & 0x3)); //set lcd bias ratio

	decltype(flash_file::lcd_uc1601s_pm) lcd_uc1601s_pm;
	fseek(p_flash, (int) (&p_flash_file->lcd_uc1601s_pm) - (int) (p_flash_file), SEEK_SET);
	fread(&lcd_uc1601s_pm, sizeof(flash_file::lcd_uc1601s_pm), 1, p_flash);

	lcd_write_control(0x81); //set Vbias potentiometer
	lcd_write_control(lcd_uc1601s_pm & 0xff);

	lcd_write_control(0x2f); //set power control 15~24nF, internal Vlcd

	lcd_write_control(0x40); //set scroll line to 0

	lcd_write_control(0x84); //disable partial display

	lcd_write_control(0x89); //increase to right, and restart in next page

	lcd_write_control(0xa0); //80 fps

	lcd_write_control(0xa4); //disable all pixel on

	lcd_write_control(0xa6); //disable inverse display

	lcd_write_control(0xc0); //disable mirror

	lcd_write_control(0xf1); //set com end
	lcd_write_control(0x3f); //set com to 64

	lcd_write_control(0xaf); //set display enable

	lcd_write_control(0x00); //set column address
	lcd_write_control(0x10);

	lcd_write_control(0xb0); //set page address

	GP0SET_SET4_BBA = true;

	lcd_tty_write(zeros, 128);

	lcd_tty_lseek(0, SEEK_SET);

	lcd_blinking(0, nullptr);
}

