//
// BitBank Capacitive Touch Sensor Library
// written by Larry Bank
//
// Copyright 2023 BitBank Software, Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================

//
// Written for the many variants of ESP32 + Capacitive touch LCDs on the market
//
#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#else
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP INPUT
#define LOW 0
#define HIGH 1
#ifdef __LINUX__
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#else // must be ESP-IDF
#include "driver/gpio.h"
#include "driver/i2c.h"
#endif // __LINUX__
#endif // ARDUINO

#ifndef __BB_CAPTOUCH__
#define __BB_CAPTOUCH__

#define CT_SUCCESS 0
#define CT_ERROR -1
//
// Pre-configured device names
// Don't change the order of this list!
// the structure values follow it. Always
// add new values to the end
//
enum {
  TOUCH_T_QT_C6=0,
  TOUCH_CYD_550,
  TOUCH_T_DISPLAY_S3_PRO,
  TOUCH_T_DISPLAY_S3_LONG,
  TOUCH_CYD_22C,
  TOUCH_CYD_24C,
  TOUCH_CYD_128,
  TOUCH_CYD_35C,
  TOUCH_CYD_518,
  TOUCH_CYD_543,
  TOUCH_M5_CORE2,
  TOUCH_M5_CORES3,
  TOUCH_M5_PAPER,
  TOUCH_WT32_SC01_PLUS,
  TOUCH_MAKERFABS_480x480,
  TOUCH_MAKERFABS_320x480,
  TOUCH_T_DISPLAY_S3_AMOLED,
  TOUCH_WS_AMOLED_18,
  TOUCH_M5_PAPERS3,
  TOUCH_WS_ROUND_146,
  TOUCH_WS_AMOLED_241,
  TOUCH_WS_LCD_169,
  TOUCH_VIEWE_2432,
  TOUCH_T_DISPLAY_S3_AMOLED_164,
  TOUCH_VPLAYER,
  TOUCH_COUNT
};
// structure holding the configurations
typedef struct bbct_config_tag {
  int8_t i8SDA, i8SCL, i8IRQ, i8RST;
} BBCT_CONFIG;

enum {
  CT_TYPE_UNKNOWN = 0,
  CT_TYPE_FT6X36,
  CT_TYPE_GT911,
  CT_TYPE_CST820,
  CT_TYPE_CST226,
  CT_TYPE_MXT144,
  CT_TYPE_AXS15231,
  CT_TYPE_TMA445,
  CT_TYPE_SPD2010,
  CT_TYPE_CHSC6540,
  CT_TYPE_COUNT
};

#define SPD2010_ADDR 0x53
#define GT911_ADDR1 0x5D
#define GT911_ADDR2 0x14
#define FT6X36_ADDR1 0x38
#define FT6X36_ADDR2 0x48
#define CST820_ADDR 0x15
#define CST226_ADDR 0x5A
#define CHSC6540_ADDR 0x2E
#define MXT144_ADDR 0x4A
#define TMA445_ADDR 0x24
#define AXS15231_ADDR 0x3B

// CST8xx gestures
enum {
  GESTURE_NONE = 0,
  GESTURE_SWIPE_UP,
  GESTURE_SWIPE_DOWN,
  GESTURE_SWIPE_LEFT,
  GESTURE_SWIPE_RIGHT,
  GESTURE_SINGLE_CLICK,
  GESTURE_DOUBLE_CLICK = 0x0B,
  GESTURE_LONG_PRESS = 0x0C
};

// TMA445 Security KEY
static uint8_t tma445_key[] = {0x00, 0x00, 0xFF, 0xA5, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    
/* TrueTouch Standard Product Gen3 (Txx3xx) interface definition */
typedef struct cyttsp_xydata_tag {
	uint8_t hst_mode;
	uint8_t tt_mode;
	uint8_t tt_stat;
	uint16_t x1 __attribute__ ((packed));
	uint16_t y1 __attribute__ ((packed));
	uint8_t z1;
	uint8_t touch12_id;
	uint16_t x2 __attribute__ ((packed));
	uint16_t y2 __attribute__ ((packed));
	uint8_t z2;
} cyttsp_xydata;
#define CY_HNDSHK_BIT 0x80
typedef struct cyttsp_bootloader_data_tag {
	uint8_t bl_file;
	uint8_t bl_status;
	uint8_t bl_error;
	uint8_t blver_hi;
	uint8_t blver_lo;
	uint8_t bld_blver_hi;
	uint8_t bld_blver_lo;
	uint8_t ttspver_hi;
	uint8_t ttspver_lo;
	uint8_t appid_hi;
	uint8_t appid_lo;
	uint8_t appver_hi;
	uint8_t appver_lo;
	uint8_t cid_0;
	uint8_t cid_1;
	uint8_t cid_2;
} cyttsp_bootloader_data;


typedef struct mxt_data_tag {
    uint16_t t2_encryption_status_address;
    uint16_t t5_message_processor_address;
    uint16_t t5_max_message_size;
    uint16_t t6_command_processor_address;
    uint16_t t7_powerconfig_address;
    uint16_t t8_acquisitionconfig_address;
    uint16_t t44_message_count_address;
    uint16_t t46_cte_config_address;
    uint16_t t100_multiple_touch_touchscreen_address;
    uint16_t t100_first_report_id;
} MXTDATA;

typedef struct mxt_object_tag {
    uint8_t type;
    uint16_t position;
    uint8_t size_minus_one;
    uint8_t instances_minus_one;
    uint8_t report_ids_per_instance;
} MXTOBJECT;

#define MXT_MESSAGE_SIZE 6

// CST820 registers
#define CST820_TOUCH_REGS 1

// GT911 registers
#define GT911_POINT_INFO 0x814E
#define GT911_POINT_1    0x814F
#define GT911_CONFIG_FRESH 0x8100
#define GT911_CONFIG_SIZE 0xb9
#define GT911_CONFIG_START 0x8047
#define GT911_ENTER_SLEEP 0x8040

// FT6x36 registers
#define TOUCH_REG_STATUS 0x02
#define TOUCH_REG_XH 0x03
#define TOUCH_REG_XL 0x04
#define TOUCH_REG_YH 0x05
#define TOUCH_REG_YL 0x06
#define TOUCH_REG_WEIGHT 0x07
#define TOUCH_REG_AREA 0x08
// register offset to info for the second touch point
#define PT2_OFFSET 6

#ifndef __TOUCHINFO_STRUCT__
#define __TOUCHINFO_STRUCT__

typedef struct _fttouchinfo
{
  int count;
  uint16_t x[5], y[5];
  uint8_t pressure[5], area[5];
} TOUCHINFO;
#endif

//extern TwoWire* myWire;

class BBCapTouch
{
public:
    BBCapTouch() { _iOrientation = 0; _iType = CT_TYPE_UNKNOWN;}
#ifndef __LINUX__
    ~BBCapTouch() { myWire->end(); }
#endif

#ifdef ARDUINO
    int init(int iConfigName);
    int init(int iSDA, int iSCL, int iRST=-1, int iINT=-1, uint32_t u32Speed=400000, TwoWire* _myWire=&Wire);
#else
    int init(int iSDA, int iSCL, int iRST=-1, int iINT=-1, uint32_t u32Speed=400000);
#endif
    int getSamples(TOUCHINFO *pTI);
    uint8_t interruptPin(void) {return (uint8_t)_iINT;}
    int sensorType(void);
    int setOrientation(int iOrientation, int iWidth, int iHeight);
    int sleep(void);
    int wake(void);

protected:
    void reset(int iResetPin);
 
private:
    int _iAddr;
    int _iType;
    int _iOrientation, _iWidth, _iHeight;
    MXTDATA _mxtdata;
    int _iINT; // interrupt GPIO pin needed for sleep/wake of GT911
    int _iRST; // reset GPIO needed for AXS15206 to wake from deep sleep
#ifdef __LINUX__
    int _file_i2c;
    void delay(int iMS);
    int digitalRead(uint8_t u8Pin);
#endif
#ifdef ARDUINO
    TwoWire* myWire;
#else
    void pinMode(uint8_t u8Pin, uint8_t u8Mode);
    void digitalWrite(uint8_t u8Pin, uint8_t u8State);
#endif
    int initMXT(void);
    void fixSamples(TOUCHINFO *pTI);
    bool I2CTest(uint8_t u8Addr);
    int I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen);
    int I2CReadRegister(uint8_t u8Addr, uint8_t u8Register, uint8_t *pData, int iLen);
    int I2CReadRegister16(uint8_t u8Addr, uint16_t u16Register, uint8_t *pData, int iLen);
    int I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen);
    void SPD2010ClearInt(void);
    void SPD2010CPUStart(void);
    uint8_t SPD2010Status(int *iNextLen);
    void SPD2010TouchStart(void);
    void SPD2010PointMode(void);
}; // class BBCapTouch
#endif // __BB_CAPTOUCH__
