#include <bb_captouch.h>

BBCapTouch bbct;
// Waveshare 2.13 epaper w/touch HAT
// SDA is really the I2C bus number on Linux (1 for Raspberry Pi)
#define TOUCH_SDA 1
#define TOUCH_SCL 0
#define TOUCH_RST 22
#define TOUCH_INT 27

const char *szNames[] = {"Unknown", "FT6x36", "GT911", "CST820", "CST226", "MXT144", "AXS15231"};

int main(int argc, char *argv[])
{
int i, iType;
TOUCHINFO ti;

  printf("Starting...\n");
  bbct.init(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, TOUCH_INT);
  iType = bbct.sensorType();
  if (iType <= 0) {
      printf("Error initalizing touch controller; exiting...\n");
      return -1;
  }
  printf("Sensor type = %s\n", szNames[iType]);

  while (1) {
      if (bbct.getSamples(&ti)) {
          for (i=0; i<ti.count; i++){
              printf("Touch %d %d,%d\n", i+1, ti.x[i], ti.y[i]);
          } // for each touch point
      } // if touch event happened
  } // while (1)
  return 0;
} /* main() */
