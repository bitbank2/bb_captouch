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
//
// Initialize the library
// It only needs to initialize the I2C interface; the chip is ready
//
int BBCapTouch::init(int iSDA, int iSCL, int iRST, int iINT, uint32_t u32Speed)
{
uint8_t ucTemp[4];

    Wire.begin(iSDA, iSCL);
    Wire.setClock(u32Speed);
    _iType = -1;

    if (I2CTest(GT911_ADDR1)) {
       _iType = CT_TYPE_GT911;
       _iAddr = GT911_ADDR1;
    } else if (I2CTest(GT911_ADDR2)) {
       _iType = CT_TYPE_GT911;
       _iAddr = GT911_ADDR2;
    }
    if (_iType == CT_TYPE_GT911) {
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
    } else if (I2CTest(FT6X36_ADDR)) {
       _iType = CT_TYPE_FT6X36;
       _iAddr = FT6X36_ADDR;
    } else {
       Wire.end();
       return CT_ERROR; // no device found
    }
    return CT_SUCCESS;
} /* init() */

bool BBCapTouch::I2CTest(uint8_t u8Addr)
{

  // Check if a device acknowledges the address.
  Wire.beginTransmission(u8Addr);
  return(Wire.endTransmission(true) == 0);
} /* I2CTest() */
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int BBCapTouch::I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen)
{
  int rc = 0;

    Wire.beginTransmission(u8Addr);
    Wire.write(pData, (uint8_t)iLen);
    rc = !Wire.endTransmission();
    return rc;
} /* I2CWrite() */
//
// Read N bytes starting at a specific 16-bit I2C register
//
int BBCapTouch::I2CReadRegister16(uint8_t u8Addr, uint16_t u16Register, uint8_t *pData, int iLen)
{
  int i = 0;

  Wire.beginTransmission(u8Addr);
  Wire.write((uint8_t)(u16Register>>8)); // high byte
  Wire.write((uint8_t)u16Register); // low byte
  Wire.endTransmission();
  Wire.requestFrom(u8Addr, (uint8_t)iLen);
  while (i < iLen)
  {
      pData[i++] = Wire.read();
  }
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

  Wire.beginTransmission(u8Addr);
  Wire.write(u8Register);
  Wire.endTransmission();
  Wire.requestFrom(u8Addr, (uint8_t)iLen);
  while (i < iLen)
  {
      pData[i++] = Wire.read();
  }
  return i;
} /* I2CReadRegister() */
//
// Read N bytes
//
int BBCapTouch::I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen)
{
  int rc;
  int i = 0;

  Wire.requestFrom(u8Addr, (uint8_t)iLen);
  while (i < iLen)
  {
     pData[i++] = Wire.read();
  }
  return i;
} /* I2CRead() */
//
// Read the touch points
// returns 0 (none), 1 for touch points available
//
int BBCapTouch::getSamples(TOUCHINFO *pTI)
{
uint8_t ucTemp[16];
int i, rc;
    
    if (!pTI)
       return 0;
    if (_iType == CT_TYPE_FT6X36) {
       rc = I2CReadRegister(_iAddr, TOUCH_REG_STATUS, ucTemp, 1); // read touch status
       if (rc == 0) { // something went wrong
           return 0;
       }
       pTI->count = 0;
       i = ucTemp[0]; // number of touch points available
       if (i >= 1) { // get data
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
         return (i > 0);
      }
    } // GT911
    return 0;
} /* getSamples() */
