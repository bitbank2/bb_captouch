//
// BitBank Capactive Touch Sensor Library
// Written by Larry Bank
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
#include "bb_captouch.h"

#ifdef __LINUX__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <gpiod.h>
#include <math.h>
#include <time.h>
#ifndef CONSUMER
#define CONSUMER "Consumer"
#endif
struct gpiod_chip *chip = NULL;
#ifdef GPIOD_API
struct gpiod_line *lines[64];
#else
struct gpiod_line_request *lines[64];
#endif
#endif // __LINUX__

static const BBCT_CONFIG _configs[] = {
       // sda, scl, irq, rst
       {21, 22, 7, 23}, // TOUCH_T_QT_C6
       {19, 20, -1, 38}, // TOUCH_CYD_550
       {5, 6, 7, 13}, // T-Display S3 Pro
       {15, 20, 11, 16}, // T-Display-S3-Long
       {21, 22, -1, -1}, // CYD_22C
       {33, 32, 21, 25}, // CYD_24C
       {4, 5, 0, 1}, // CYD_128
       {33, 32, 21, 25}, // CYD_35C
       {7, 8, 41, 40}, // CYD_518
       {8, 4, 3, -1}, // CYD_543
       {21, 22, 39, -1}, // M5_CORE2
       {12, 11, -1, -1}, // M5_CORES3
       {21, 22, 36, -1}, // M5_PAPER
       {6, 5 ,7, -1}, // WT32_SC01_PLUS
       {17, 18, -1, 38}, // MakerFabs 4" 480x480
       {38,39,40,-1}, // MakerFabs 3.5" 320x480
       {7, 6, 9, 8}, // TOUCH_T_DISPLAY_S3_AMOLED (1.64")
       {15, 14, 21, -1}, // TOUCH_WS_AMOLED_18 (Waveshare 1.8" 368x448 AMOLED)
       {41, 42, 48, -1}, // TOUCH_M5_PAPERS3
       {11, 10, 4, -1}, // TOUCH_WS_ROUND_146
       {47,48,-1,3}, // TOUCH_WS_AMOLED_241
       {11,10,14,13}, // TOUCH_WS_LCD_169
       {1,3,4,2}, // TOUCH_VIEWE_2432
       {7, 6, 9, 8}, // TOUCH_T_DISPLAY_S3_AMOLED_164
       {16,15,18,17}, // TOUCH_VPLAYER (ESP32-S3 + 1.69 280x240)
       {0,0,0,0}
    };

#ifndef ARDUINO
#ifdef __LINUX__
void BBCapTouch::delay(int iMS)
{
  usleep(iMS * 1000);
} /* delay() */ 

int BBCapTouch::digitalRead(uint8_t iPin)
{
        if (lines[iPin] == 0) return 0;
#ifdef GPIOD_API // 1.x (old) API
  return gpiod_line_get_value(lines[iPin]);
#else // 2.x (new)
  return gpiod_line_request_get_value(lines[iPin], iPin) == GPIOD_LINE_VALUE_ACTIVE;
#endif
} /* digitalRead() */

void BBCapTouch::pinMode(uint8_t iPin, uint8_t iMode)
{
#ifdef GPIOD_API // old 1.6 API
   if (chip == NULL) {
       chip = gpiod_chip_open_by_name("gpiochip0");
   }
   lines[iPin] = gpiod_chip_get_line(chip, iPin);
   if (iMode == OUTPUT) {
       gpiod_line_request_output(lines[iPin], CONSUMER, 0);
   } else if (iMode == INPUT_PULLUP) {
       gpiod_line_request_input_flags(lines[iPin], CONSUMER, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
   } else { // plain input
       gpiod_line_request_input(lines[iPin], CONSUMER);
   }
#else // new 2.x API
   struct gpiod_line_settings *settings;
   struct gpiod_line_config *line_cfg;
   struct gpiod_request_config *req_cfg;
   chip = gpiod_chip_open("/dev/gpiochip0");
   if (!chip) {
        printf("chip open failed\n");
           return;
   }
   settings = gpiod_line_settings_new();
   if (!settings) {
        printf("line_settings_new failed\n");
           return;
   }
   gpiod_line_settings_set_direction(settings, (iMode == OUTPUT) ? GPIOD_LINE_DIRECTION_OUTPUT : GPIOD_LINE_DIRECTION_INPUT);
   line_cfg = gpiod_line_config_new();
   if (!line_cfg) return;
   gpiod_line_config_add_line_settings(line_cfg, (const unsigned int *)&iPin, 1, settings);
   req_cfg = gpiod_request_config_new();
   gpiod_request_config_set_consumer(req_cfg, CONSUMER);
   lines[iPin] = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
   gpiod_request_config_free(req_cfg);
   gpiod_line_config_free(line_cfg);
   gpiod_line_settings_free(settings);
   gpiod_chip_close(chip);
#endif
} /* pinMode() */
void BBCapTouch::digitalWrite(uint8_t iPin, uint8_t iState)
{
        if (lines[iPin] == 0) return;
#ifdef GPIOD_API // old 1.6 API
   gpiod_line_set_value(lines[iPin], iState);
#else // new 2.x API
   gpiod_line_request_set_value(lines[iPin], iPin, (iState) ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
#endif
} /* digitalWrite() */
#else // must be ESP-IDF
void BBCapTouch::pinMode(uint8_t u8Pin, uint8_t u8Mode)
{
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1 << u8Pin);
    io_conf.pull_down_en = 0; //disable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    if (u8Mode == INPUT) {
        io_conf.mode = GPIO_MODE_INPUT;
    } else { // must be output
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    gpio_config(&io_conf); //configure GPIO with the given settings
} /* pinMode() */
void BBCapTouch::digitalWrite(uint8_t u8Pin, uint8_t u8State)
{
    gpio_set_level(u8Pin, u8State);
} /* digitalWrite() */
#endif // __LINUX__
#endif // !ARDUINO
void BBCapTouch::reset(int iRST)
{
     pinMode(iRST, OUTPUT);
     digitalWrite(iRST, LOW);
     delay(100);
     digitalWrite(iRST, HIGH);
     delay(250);
}  /* reset() */

//
// Initalize the MXT144 - an overly complicated mess of a touch sensor
//
int BBCapTouch::initMXT(void)
{
uint8_t ucTemp[32];
int i, iObjCount, iReportID;
uint16_t u16, u16Offset;

// Read information block (first 7 bytes of address space)
   I2CReadRegister16(_iAddr, 0, ucTemp, 7);
   iObjCount = ucTemp[6];
   if (iObjCount < 1 || iObjCount >64) { // invalid number of items
      return CT_ERROR;
   }
   u16Offset = 7; // starting offset of first object
   // Read the objects one by one to get the memory offests to the info we will need
   iReportID = 1;
  // Serial.printf("object count = %d\n", iObjCount);
   for (i=0; i<iObjCount; i++) {
      I2CReadRegister16(_iAddr, u16Offset, ucTemp, 6);
      //Serial.printf("object %d, type %d\n", i, ucTemp[0]);
      u16 = ucTemp[1] | (ucTemp[2] << 8);
      switch (ucTemp[0]) {
         case 2:
            _mxtdata.t2_encryption_status_address = u16;
            break;
         case 5:
            _mxtdata.t5_message_processor_address = u16;
            _mxtdata.t5_max_message_size = ucTemp[4] - 1;
            break;
         case 6:
            _mxtdata.t6_command_processor_address = u16;
            break;
         case 7:
            _mxtdata.t7_powerconfig_address = u16;
            break;
         case 8:
            _mxtdata.t8_acquisitionconfig_address = u16;
            break;
         case 44:
            _mxtdata.t44_message_count_address = u16;
            break;
         case 46:
            _mxtdata.t46_cte_config_address = u16;
            break;
         case 100:
            _mxtdata.t100_multiple_touch_touchscreen_address = u16;
            _mxtdata.t100_first_report_id = iReportID;
            break;
         default:
            break;
      } // switch on type
      u16Offset += 6; // size in bytes of an object table
      iReportID += ucTemp[5] * (ucTemp[4] + 1);
   } // for each report
//Serial.printf("init success, count offset = %d\n", _mxtdata.t44_message_count_address);
   return CT_SUCCESS;
} /* initMXT() */

//
// Initialize the library
// It only needs to initialize the I2C interface; the chip is ready
//
#ifdef ARDUINO
int BBCapTouch::init(int iSDA, int iSCL, int iRST, int iINT, uint32_t u32Speed, TwoWire* _myWire)
#else
int BBCapTouch::init(int iSDA, int iSCL, int iRST, int iINT, uint32_t u32Speed)
#endif
{
uint8_t ucTemp[4];

    _iINT = iINT; // save for sleep/wake of GT911
    _iRST = iRST; // save for sleep/wake of AXS15231
#ifdef ARDUINO
    myWire = _myWire;
    myWire->begin(iSDA, iSCL); // this is specific to ESP32 MCUs
    myWire->setClock(u32Speed);
    myWire->setTimeout(1000);
#else
#ifdef __LINUX__
char filename[32];
int iChannel = iSDA;
    sprintf(filename, "/dev/i2c-%d", iChannel);
    if ((_file_i2c = open(filename, O_RDWR)) < 0) {
        fprintf(stderr, "Failed to open the i2c bus\n");
    }
#else // must be ESP-IDF
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = iSDA,
        .scl_io_num = iSCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = u32Speed,
    };
    i2c_driver_delete(I2C_NUM_0); // remove driver (if installed)
    i2c_param_config(I2C_NUM_0, &conf); // configure I2C device 0
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0); // configure with no send or receive buffers
#endif // __LINUX__
#endif
    _iType = CT_TYPE_UNKNOWN;

    if (iRST != -1) {
       reset(iRST);
    }

#ifdef FUTURE
    if (I2CTest(AXS15231_ADDR)) {
       _iType = CT_TYPE_AXS15231;
       _iAddr = AXS15231_ADDR;
       if (iRST != -1) {
          reset(iRST);
       }
       return CT_SUCCESS;
    } // AXS15231
#endif

    if (I2CTest(SPD2010_ADDR)) {
        _iType = CT_TYPE_SPD2010;
        _iAddr = SPD2010_ADDR;
        if (iINT != -1) {
            pinMode(iINT, INPUT_PULLUP);
        }
        return CT_SUCCESS;
    }
    if (I2CTest(TMA445_ADDR)) {
       _iType = CT_TYPE_TMA445;
       _iAddr = TMA445_ADDR;
//       if (iINT != -1) {
//          pinMode(iINT, INPUT_PULLUP);
//       }
       // send soft reset
       ucTemp[0] = 0;
       ucTemp[1] = 1;
       I2CWrite(_iAddr, ucTemp, 2);
       delay(50);
       // send security key
       I2CWrite(_iAddr, tma445_key, sizeof(tma445_key));
       delay(88);

       uint8_t tries = 0;
       cyttsp_bootloader_data  bl_data = {};
       do {
        delay(20);
        I2CRead(_iAddr, (uint8_t *)&bl_data, sizeof(bl_data));
       } while (bl_data.bl_status & 0x10 && tries++ < 10); // while bootloader mode
       if (tries >= 10) {
       //    Serial.println("bootloader mode timed out");
       }
       // set OP mode
       ucTemp[0] = 0;
       ucTemp[1] = 0; // start op mode
       I2CWrite(_iAddr, ucTemp, 2);
       return CT_SUCCESS;
    }
    if (I2CTest(MXT144_ADDR)) {
       _iType = CT_TYPE_MXT144;
       _iAddr = MXT144_ADDR;
       if (iINT != -1) {
          pinMode(iINT, INPUT);
       }
       if (iRST != -1) {
          reset(iRST);
       }
       return initMXT();
    }
    if (I2CTest(CHSC6540_ADDR)) {
       _iType = CT_TYPE_CHSC6540;
       _iAddr = CHSC6540_ADDR;
       if (iINT != -1) {
          pinMode(iINT, INPUT);
       }
       ucTemp[0] = ucTemp[1] = 0x5a; // set interrupt mode
       I2CWrite(_iAddr, ucTemp, 2);
       return CT_SUCCESS;
    }
    if (I2CTest(CST226_ADDR)) {
       _iType = CT_TYPE_CST226;
       _iAddr = CST226_ADDR;
       if (iINT != -1) {
          pinMode(iINT, INPUT);
       }
       return CT_SUCCESS;
    }
    if (I2CTest(GT911_ADDR1) || I2CTest(GT911_ADDR2)) {
       _iType = CT_TYPE_GT911;
    }
    if (_iType == CT_TYPE_GT911) { // reset the sensor to start it
       pinMode(iRST, OUTPUT);
       pinMode(iINT, OUTPUT);
       digitalWrite(iINT, LOW);
       digitalWrite(iRST, LOW);
       delay(5);
       digitalWrite(iINT, LOW); // set I2C addr to ADDR1
       delay(1);
       digitalWrite(iRST, HIGH); // when it comes out of reset, it samples INT
       delay(10);
       digitalWrite(iINT, LOW);
       delay(50);
       pinMode(iINT, INPUT);
       // double check the I2C addr in case it changed
       if (I2CTest(GT911_ADDR1)) {
          _iAddr = GT911_ADDR1;
       } else if (I2CTest(GT911_ADDR2)) {
          _iAddr = GT911_ADDR2;
       }
    } else if (I2CTest(FT6X36_ADDR1)) {
       _iType = CT_TYPE_FT6X36;
       _iAddr = FT6X36_ADDR1;
    } else if (I2CTest(FT6X36_ADDR2)) {
       _iType = CT_TYPE_FT6X36;
       _iAddr = FT6X36_ADDR2;
    } else if (I2CTest(CST820_ADDR)) {
       _iType = CT_TYPE_CST820;
       _iAddr = CST820_ADDR;
       if (iRST != -1) {
          reset(iRST);
       }
    } else {
#ifdef ARDUINO
       myWire->end();
#else
#ifdef __LINUX__
       close(_file_i2c);
#else
       i2c_driver_delete(I2C_NUM_0);
#endif // __LINUX__
#endif
       return CT_ERROR; // no device found
    }
    return CT_SUCCESS;
} /* init() */

#ifndef __LINUX__
// Initialize the touch controller from a pre-defined configuration name
int BBCapTouch::init(int iConfigName)
{
const BBCT_CONFIG *pC = &_configs[iConfigName];

   if (iConfigName < 0 || iConfigName >= TOUCH_COUNT) {
       return CT_ERROR;
   }

   if (iConfigName == TOUCH_WS_AMOLED_241) { // touch is rotated
       _iOrientation = 270;
       _iWidth = 450;
       _iHeight = 600;

   }
   if (iConfigName == TOUCH_WS_AMOLED_18 || iConfigName == TOUCH_WS_ROUND_146) {
       // need to release the RESET line which is controlled by an
       // I/O expander (TCA9554)
       uint8_t ucEXIO;
       if (iConfigName == TOUCH_WS_AMOLED_18) ucEXIO = 6; // touch reset (AMOLED 1.8)
       else ucEXIO = 1; // touch reset (1.46" round)
       Wire.end();
       Wire.begin(pC->i8SDA, pC->i8SCL);
       Wire.beginTransmission(0x20);
       Wire.write(1); // output port register
       Wire.write(~ucEXIO); // set touch controller reset low
       Wire.write(0); // polarity inversion all disabled
       Wire.write(~7); // enable P0+P1+P2 as an output (connected to RESET)
       Wire.endTransmission();
       delay(50);
       Wire.beginTransmission(0x20);
       Wire.write(1); // output port register
       Wire.write(0xff); // set all outputs high (disable resets)
       Wire.endTransmission();
       delay(50);
       if (iConfigName == TOUCH_WS_AMOLED_18) {
           Wire.beginTransmission(0x38);
           Wire.write(0xa5); // power mode
           Wire.write(0); // active
           Wire.endTransmission();
       }
       Wire.end();
   }
   return init(pC->i8SDA, pC->i8SCL, pC->i8RST, pC->i8IRQ,
#ifdef ARDUINO
               400000, &Wire);
#else
               400000);
#endif
} /* init() */
#endif
//
// Test if an I2C device is monitoring an address
// return true if it responds, false if no response
//
bool BBCapTouch::I2CTest(uint8_t u8Addr)
{
  // Check if a device acknowledges the address.
#ifdef ARDUINO
  myWire->beginTransmission(u8Addr);
  return(myWire->endTransmission(true) == 0);
#else // allow 100ms for device to respond
#ifdef __LINUX__
uint8_t response = 0;

    if (ioctl(_file_i2c, I2C_SLAVE, u8Addr) >= 0) {
        // probe this address
        uint8_t ucTemp;
        if (read(_file_i2c, &ucTemp, 1) >= 0)
            response = 1;
    }
    return response;
#else // must be ESP-IDF
    uint8_t c = 0;
    return (i2c_master_write_to_device(I2C_NUM_0, u8Addr, &c, 1, 10) == ESP_OK);
#endif // __LINUX__
#endif
} /* I2CTest() */
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int BBCapTouch::I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen)
{
  int rc = 0;

#ifdef ARDUINO
    myWire->beginTransmission(u8Addr);
    myWire->write(pData, (uint8_t)iLen);
    rc = !myWire->endTransmission();
#else
#ifdef __LINUX__
        ioctl(_file_i2c, I2C_SLAVE, u8Addr);
        rc = write(_file_i2c, pData, iLen);
        return rc;
#else // must be ESP-IDF
    rc = (i2c_master_write_to_device(I2C_NUM_0, u8Addr, pData, iLen, 100) == ESP_OK);
#endif // __LINUX__
#endif
    return rc;
} /* I2CWrite() */
//
// Read N bytes starting at a specific 16-bit I2C register
//
int BBCapTouch::I2CReadRegister16(uint8_t u8Addr, uint16_t u16Register, uint8_t *pData, int iLen)
{
  int i = 0;

#ifdef ARDUINO
  myWire->beginTransmission(u8Addr);
  if (_iType == CT_TYPE_MXT144) { // little endian
    myWire->write((uint8_t)u16Register); // low byte
    myWire->write((uint8_t)(u16Register>>8)); // high byte
  } else { // big endian address
    myWire->write((uint8_t)(u16Register>>8)); // high byte
    myWire->write((uint8_t)u16Register); // low byte
  }
  myWire->endTransmission();
  myWire->requestFrom(u8Addr, (uint8_t)iLen);
  while (i < iLen)
  {
      pData[i++] = myWire->read();
  }
#else
    uint8_t ucTemp[4];
    int rc;
    if (_iType == CT_TYPE_MXT144) { // little endian
        ucTemp[1] = (uint8_t)(u16Register>>8); // high byte
        ucTemp[0] = (uint8_t)u16Register; // low byte
    } else {
        ucTemp[0] = (uint8_t)(u16Register>>8); // high byte
        ucTemp[1] = (uint8_t)u16Register; // low byte
    }
#ifdef __LINUX__
        ioctl(_file_i2c, I2C_SLAVE, u8Addr);
        rc = write(_file_i2c, ucTemp, 2); // write the register value
        if (rc == 2)
        {
            i = read(_file_i2c, pData, iLen);
        }
#else // esp-idf
    i2c_master_write_read_device(I2C_NUM_0, u8Addr, ucTemp, 2, pData, iLen, 100);
    if (rc == ESP_OK) {
            i = iLen;
    }
#endif // __LINUX__
#endif
  return i;

} /* I2CReadRegister16() */
//
// Read N bytes starting at a specific I2C internal register
// returns 1 for success, 0 for error
//
int BBCapTouch::I2CReadRegister(uint8_t u8Addr, uint8_t u8Register, uint8_t *pData, int iLen)
{
  int rc;
  int i = 0;

#ifdef ARDUINO
  myWire->beginTransmission(u8Addr);
  myWire->write(u8Register);
  myWire->endTransmission();
  myWire->requestFrom(u8Addr, (uint8_t)iLen);
 // i = myWire->readBytes(pData, iLen);
  while (myWire->available() && i < iLen)
  {
      pData[i++] = myWire->read();
  }
#else
#ifdef __LINUX__
        ioctl(_file_i2c, I2C_SLAVE, u8Addr);
        rc = write(_file_i2c, &u8Register, 1); // write the register value
        if (rc == 1)
        {
                i = read(_file_i2c, pData, iLen);
        }
#else // esp-idf
    rc = i2c_master_write_read_device(I2C_NUM_0, u8Addr, &u8Register, 1, pData, iLen, 100);
    i = (rc == ESP_OK);
#endif // __LINUX__
#endif
  return i;
} /* I2CReadRegister() */
//
// Read N bytes
//
int BBCapTouch::I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen)
{
  int i = 0;

#ifdef ARDUINO
  myWire->requestFrom(u8Addr, (uint8_t)iLen);
  while (i < iLen)
  {
     pData[i++] = myWire->read();
  }
#else
#ifdef __LINUX__
        ioctl(_file_i2c, I2C_SLAVE, u8Addr);
        i = read(_file_i2c, pData, iLen);
#else // esp-idf
    int rc;
    rc = i2c_master_read_from_device(I2C_NUM_0, u8Addr, pData, iLen, 100);
    i = (rc == ESP_OK) ? iLen : 0;
#endif // __LINUX__
#endif
  return i;
} /* I2CRead() */
//
// Private function to rotate touch samples if the user
// specified a new display orientation
//
void BBCapTouch::fixSamples(TOUCHINFO *pTI)
{
int i, x, y;

   for (i=0; i<pTI->count; i++) {
       switch (_iOrientation) {
           case 90:
               x = pTI->y[i];
               y = _iWidth - 1 - pTI->x[i];
               pTI->x[i] = x;
               pTI->y[i] = y;
               break;
           case 180:
               pTI->x[i] = _iWidth - 1 - pTI->x[i];
               pTI->y[i] = _iHeight - 1 - pTI->y[i];
               break;
           case 270:
               x = _iHeight - 1 - pTI->y[i];
               y = pTI->x[i];
               pTI->x[i] = x;
               pTI->y[i] = y;
               break;
           default: // do nothing
               break;
       }
   }
} /* fixSamples() */
void BBCapTouch::SPD2010ClearInt(void)
{
    uint8_t ucTemp[4];
    ucTemp[0] = 2;
    ucTemp[1] = 0;
    ucTemp[2] = 1;
    ucTemp[3] = 0;
    I2CWrite(_iAddr, ucTemp, 4);
} /* SPD2010ClearInt() */

void BBCapTouch::SPD2010CPUStart(void)
{
    uint8_t ucTemp[4];
    ucTemp[0] = 4;
    ucTemp[1] = 0;
    ucTemp[2] = 1;
    ucTemp[3] = 0;
    I2CWrite(_iAddr, ucTemp, 4);
} /* SPD2010CPUStart() */

uint8_t BBCapTouch::SPD2010Status(int *iNextLen)
{
    uint8_t ucTemp[8];
    I2CReadRegister16(_iAddr, 0xfc02, ucTemp, 8);
    *iNextLen = (ucTemp[3] << 8) | ucTemp[2];
    return ucTemp[5];
} /* SPD2010Status() */

void BBCapTouch::SPD2010TouchStart(void)
{
    uint8_t ucTemp[4];
    ucTemp[0] = 0x46;
    ucTemp[1] = 0x0;
    ucTemp[2] = 0x0;
    ucTemp[3] = 0x0;
    I2CWrite(_iAddr, ucTemp, 4);
} /* SPD2010TouchStart() */

void BBCapTouch::SPD2010PointMode(void)
{
    uint8_t ucTemp[4];
    ucTemp[0] = 0x50;
    ucTemp[1] = 0x0;
    ucTemp[2] = 0x0;
    ucTemp[3] = 0x0;
    I2CWrite(_iAddr, ucTemp, 4);
} /* SPD2010PointMode() */

//
// Read the touch points
// returns 0 (none), 1 if touch points are available
// The point count and info is returned in the TOUCHINFO structure
//
int BBCapTouch::getSamples(TOUCHINFO *pTI)
{
uint8_t c, *s, ucTemp[32];
int i, j, rc = 0;
    
    if (!pTI)
       return 0;
    pTI->count = 0;
    if (_iType == CT_TYPE_UNKNOWN) return 0; // library not initialized?

    if (_iType == CT_TYPE_SPD2010) {
        static int iNextLen = 0;
        int iReadLen;
        uint8_t ucStatus;
        // first read the status
        I2CReadRegister16(_iAddr, 0x2000, ucTemp, 4);
        // Byte 0 has a bit field with the following info:
        // 0 = point exists
        // 1 = gesture
        // 3 = aux, cytang
        // Byte 1
        // 3 = cpu run
        // 4 = tint low
        // 5 = tic in cpu
        // 6 = tic in bios
        // 7 = tic busy
        iReadLen = (ucTemp[3]<<8) + ucTemp[2];
       // Serial.printf("status byte 0: 0x%02x, 0x%02x, read len = %d\n", ucTemp[0], ucTemp[1], iReadLen);
        if (ucTemp[1] & 0x40) { // tic in bios
            SPD2010ClearInt(); // Write Clear TINT Command
            SPD2010CPUStart(); // Write CPU Start Command
        } else if (ucTemp[1] & 0x20) { // tic in cpu
            SPD2010PointMode(); // Write Touch Change Command
            SPD2010TouchStart(); // Write Touch Start Command
            SPD2010ClearInt();
        } else if ((ucTemp[1] & 8) && iReadLen == 0) { // cpu run
            SPD2010ClearInt();
        } else if (ucTemp[0] & 1 || ucTemp[0] & 2) { // point or gesture
            uint8_t ucData[64]; // 4-byte header plus up to 10 fingers of 6 bytes each
            I2CReadRegister16(_iAddr, 0x0003, ucData, iReadLen);
         //   Serial.printf("readlen: %d, data[4]: 0x%02x\n", iReadLen, ucData[4]);
            if (ucData[4] <= 0xa && ucTemp[0] & 1) { // check_id < 10 && point exists
                rc = 1; // touch event
                pTI->count = (iReadLen - 4)/6; // number of touch points
                for (int i=0; i<pTI->count; i++) {
                    int iOffset = i*6;
                    pTI->x[i] = ((ucData[7+iOffset] & 0xf0) << 4) | ucData[5+iOffset];
                    pTI->y[i] = ((ucData[7+iOffset] & 0x0f) << 8) | ucData[6+iOffset];
                    pTI->area[i] = ucData[8+iOffset];
                }
            }
            // gestures...
      hdp_done_check:
          /* Read HDP Status */
            ucStatus = SPD2010Status(&iNextLen);
//            Serial.printf("status: 0x%02\n", ucStatus);
            if (ucStatus == 0x82) {
                SPD2010ClearInt();
            } else if (ucStatus == 0x00) { // Read HDP Remain Data
               // Read_HDP_REMAIN_DATA(&tp_hdp_status);
                goto hdp_done_check;
            }
        } else if (ucTemp[1] & 8 && ucTemp[0] & 8) { // cpu run && aux
            SPD2010ClearInt();
        }
        return rc;
    }
    if (_iType == CT_TYPE_TMA445) {
        uint8_t hst_mode;
        I2CReadRegister(_iAddr, 0, ucTemp, 14); // read up to 2 touch points
        pTI->count = ucTemp[2]; // touch point count
        if (pTI->count > 0) {
            pTI->y[0] = (ucTemp[5]*256) + ucTemp[6];
            pTI->x[0] = (ucTemp[3]*256) + ucTemp[4];
            pTI->area[0] = ucTemp[7];
            if (pTI->count == 2) { // get second touch point
                pTI->y[1] = (ucTemp[11]*256) + ucTemp[12];
                pTI->x[1] = (ucTemp[9]*256) + ucTemp[10];
                pTI->area[1] = ucTemp[13];
            }
            // do handshake
            I2CReadRegister(_iAddr, 0, &hst_mode, 1);
            hst_mode ^= CY_HNDSHK_BIT; // toggle handshake bit
            ucTemp[0] = 0;
            ucTemp[1] = hst_mode; // send it back
            I2CWrite(_iAddr, ucTemp, 2);
            return 1;
        }
        return 0;
    }
    if (_iType == CT_TYPE_MXT144) {
       if (!_mxtdata.t44_message_count_address) {
          return 0; // No message offset, so we can't read anything :(
       }
       I2CReadRegister16(_iAddr, _mxtdata.t44_message_count_address, ucTemp, 1);
       j = ucTemp[0]; // object count
       // As each message is read from the sensor, the internal count
       // is decremented. It appears that it can hold 6 messages maximum
       // before you must read them to receive more.
       for (i = 0; i < j; i++) { // read the messages
          I2CReadRegister16(_iAddr, _mxtdata.t5_message_processor_address, ucTemp, MXT_MESSAGE_SIZE); // each message is 6 bytes
          // check report_id
          if (ucTemp[0] >= _mxtdata.t100_first_report_id + 2 &&
            ucTemp[0] < _mxtdata.t100_first_report_id + 2 + 5) {
              uint8_t finger_idx = ucTemp[0] - _mxtdata.t100_first_report_id - 2;
              uint8_t event = ucTemp[1] & 0xf;
              if (finger_idx+1 > pTI->count) pTI->count = finger_idx+1;
              pTI->x[finger_idx] = ucTemp[2] + (ucTemp[3] << 8);
              pTI->y[finger_idx] = ucTemp[4] + (ucTemp[5] << 8);
              if (event == 1 || event == 4) { // move/press event
                  pTI->area[finger_idx] = 50;
              } else if (event == 5) { // release
                  pTI->area[finger_idx] = 0;
              }
           } // if touch report
       } // for each report
       return (pTI->count > 0);
    } // MXT144

    if (_iType == CT_TYPE_AXS15231) {
        uint8_t ucReadCMD[8] = {0xb5,0xab,0xa5,0x5a,0,0,0,0x8};
        I2CWrite(_iAddr, (uint8_t *)ucReadCMD, 8);
        I2CRead(_iAddr, ucTemp, 14); // read up to 2 touch points
        c = ucTemp[1]; // number of touch points
        if (c == 0 || c > 2 || ucTemp[0] != 0) return 0;
        pTI->count = c;
        j = 0; // buffer offset
        for (i=0; i<c; i++) {
             pTI->x[i] = ((ucTemp[j+2] & 0xf) << 8) + ucTemp[j+3];
             pTI->y[i] = ((ucTemp[j+4] & 0xf) << 8) + ucTemp[j+5];
             pTI->area[i] = 1;
             j += 6; 
        }
        if (_iOrientation != 0) fixSamples(pTI);
        return 1;
    } // AXS15231
 
    if (_iType == CT_TYPE_CST226) {
        i = I2CReadRegister(_iAddr, 0, ucTemp, 28); // read the whole block of regs
//      Serial.printf("I2CReadRegister returned %d\n", i);
#ifdef FUTURE
        if (ucTemp[0] == 0x83 && ucTemp[1] == 0x17 && ucTemp[5] == 0x80) {
        // home button pressed
            return 0;
        }
        if (ucTemp[6] != 0xab) return 0;
        if (ucTemp[0] == 0xab) return 0;
        if (ucTemp[5] == 0x80) return 0;
        c = ucTemp[5] & 0x7f;
        if (c > 5 || c == 0) { // invalid point count
           ucTemp[0] = 0;
           ucTemp[1] = 0xab;
           I2CWrite(_iAddr, ucTemp, 2); // reset
           return 0;
        }
#endif
        c = 1; // debug
        pTI->count = c;
       // Serial.printf("count = %d\n", c);
        j = 0;
        for (i=0; i<c; i++) {
           pTI->x[i] = (uint16_t)((ucTemp[j+1] << 4) | ((ucTemp[j+3] >> 4) & 0xf));
           pTI->y[i] = (uint16_t)((ucTemp[j+2] << 4) | (ucTemp[j+3] & 0xf));
           pTI->pressure[i] = ucTemp[j+4];
           j = (i == 0) ? (j+7) : (j+5);
        }
        if (_iOrientation != 0) fixSamples(pTI);
        return 1;
    }

    if (_iType == CT_TYPE_CHSC6540) {
       if (digitalRead(_iINT) == LOW) {
           I2CReadRegister(_iAddr, 2, ucTemp, 13); // read touch points
           if (ucTemp[2] == 1 || ucTemp[2] == 2) {
               pTI->count = ucTemp[2];
               pTI->x[0] = ((ucTemp[3] & 0xf) << 8) | ucTemp[4];
               pTI->y[0] = ((ucTemp[5] & 0xf) << 8) | ucTemp[6];
               if (ucTemp[0] == 2) { // second touch point
                   pTI->x[0] = ((ucTemp[9] & 0xf) << 8) | ucTemp[10];
                   pTI->y[0] = ((ucTemp[11] & 0xf) << 8) | ucTemp[12];
               }  
               return 1;
           }
       }
       return 0;
    }
    if (_iType == CT_TYPE_CST820) {
       I2CReadRegister(_iAddr, CST820_TOUCH_REGS+1, ucTemp, 1); // read touch count
       if (ucTemp[0] < 1 || ucTemp[0] > 5) { // something went wrong
           return 0;
       }
       if (ucTemp[0] >= 1) { // touch data available, read it
           pTI->count = ucTemp[0];
           I2CReadRegister(_iAddr, CST820_TOUCH_REGS+2, ucTemp, pTI->count * 6);
           s = ucTemp;
           for (i=0; i<pTI->count; i++) {
              pTI->x[i] = ((s[0] & 0xf) << 8) | s[1];
              pTI->y[i] = ((s[2] & 0xf) << 8) | s[3];
              pTI->area[i] = 1; // no data available
              s += 6;
           }
           if (_iOrientation != 0) fixSamples(pTI);
           return 1;
       }
    }
    if (_iType == CT_TYPE_FT6X36) {
       rc = I2CReadRegister(_iAddr, TOUCH_REG_STATUS, ucTemp, 1); // read touch status
       if (rc == 0) { // something went wrong
           return 0;
       }
       i = ucTemp[0]; // number of touch points available
       if (i >= 1 && i <= 5) { // get data
           rc = I2CReadRegister(_iAddr, TOUCH_REG_XH, ucTemp, 6*i); // read X+Y position(s)
           if ((ucTemp[0] & 0x40) == 0 && (ucTemp[2] & 0xf0) != 0xf0) { // finger is down
               pTI->x[0] = ((ucTemp[0] & 0xf) << 8) | ucTemp[1];
               pTI->y[0] = ((ucTemp[2] & 0xf) << 8) | ucTemp[3];
               // get touch pressure and area
               pTI->pressure[0] = ucTemp[4];
               pTI->area[0] = ucTemp[5];
               pTI->count++;
           }
           if (i > 1) { // get second point
               if ((ucTemp[6] & 0x40) == 0 && (ucTemp[8] & 0xf0) != 0xf0) { // finger is down
                   pTI->x[1] = ((ucTemp[6] & 0xf) << 8) | ucTemp[7];
                   pTI->y[1] = ((ucTemp[8] & 0xf) << 8) | ucTemp[9];
                   // get touch pressure and area
                   pTI->pressure[1] = ucTemp[10];
                   pTI->area[1] = ucTemp[11];
                   pTI->count++;
               }
           }
       } // if touch points available
       if (_iOrientation != 0) fixSamples(pTI);
       return 1;
    } else { // GT911
      I2CReadRegister16(_iAddr, GT911_POINT_INFO, ucTemp, 1); // get number of touch points
      i = ucTemp[0] & 0xf; // number of touches
      if (i <= 5 && ucTemp[0] & 0x80) { // if buffer status is good + >= 1 touch points
         ucTemp[0] = (uint8_t)(GT911_POINT_INFO >> 8);
         ucTemp[1] = (uint8_t)GT911_POINT_INFO;
         ucTemp[2] = 0; // clear touch info for next time
         I2CWrite(_iAddr, ucTemp, 3);

         pTI->count = i;
         for (int j=0; j<i; j++) { // read each touch point block
             I2CReadRegister16(_iAddr, GT911_POINT_1 + (j*8), ucTemp, 7);
             pTI->x[j] = ucTemp[1] + (ucTemp[2] << 8);
             pTI->y[j] = ucTemp[3] + (ucTemp[4] << 8);
             pTI->area[j] = ucTemp[5] + (ucTemp[6] << 8);
             pTI->pressure[j] = 0; 
         }
         if (i && _iOrientation != 0) fixSamples(pTI);
         return (i > 0);
      }
    } // GT911
    return 0;
} /* getSamples() */

int BBCapTouch::setOrientation(int iOrientation, int iWidth, int iHeight)
{
    if (iOrientation != 0 && iOrientation != 90 && iOrientation != 180 && iOrientation != 270) {
       return CT_ERROR;
    }
    _iOrientation = iOrientation;
    _iWidth = iWidth;
    _iHeight = iHeight;
    return CT_SUCCESS;
} /* setOrientation() */

int BBCapTouch::sensorType(void)
{
   return _iType;
} /* type() */

int BBCapTouch::sleep(void)
{
uint8_t ucTemp[8];

    if (_iType == CT_TYPE_AXS15231) {
        ucTemp[0] = 0x90; // enter deep sleep
        ucTemp[1] = 0xa5;
        ucTemp[2] = 0x5a;
        ucTemp[3] = 0x15;
        ucTemp[4] = 0x23;
        I2CWrite(_iAddr, ucTemp, 5);
        return CT_SUCCESS;
    }
    if (_iType == CT_TYPE_GT911 && _iINT != -1) {
        pinMode(_iINT, OUTPUT);
        digitalWrite(_iINT, 0); // setting interrupt pin low to prepare sleep mode
        ucTemp[0] = (uint8_t)(GT911_ENTER_SLEEP >> 8);
        ucTemp[1] = (uint8_t)GT911_ENTER_SLEEP;
        ucTemp[2] = 0x05; //??
        I2CWrite(_iAddr, ucTemp, 3); // enter sleep mode
        return CT_SUCCESS;
    }
    if (_iType == CT_TYPE_CST226) {
        ucTemp[0] = 0xd1;
        ucTemp[1] = 0x05;
        I2CWrite(_iAddr, ucTemp, 2);
        return CT_SUCCESS;
    }
    return CT_ERROR;
} /* sleep() */

int BBCapTouch::wake(void)
{
uint8_t ucTemp[4];

    if (_iType == CT_TYPE_AXS15231) {
        digitalWrite(_iRST, 0);
        delay(50);
        digitalWrite(_iRST, 1);
        return CT_SUCCESS;
    }
    if (_iType == CT_TYPE_GT911 && _iINT != -1) {
        digitalWrite(_iINT, 1); // wake up the controller
        delay(5); // allow it time to wake
        pinMode(_iINT, INPUT); // change pin mode back to floating
    }
    if (_iType == CT_TYPE_CST226) {
        ucTemp[0] = 0xd1;
        ucTemp[1] = 0x06; 
        I2CWrite(_iAddr, ucTemp, 2);
        return CT_SUCCESS;
    }
    return CT_ERROR;
} /* wake() */


