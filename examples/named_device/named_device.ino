//
// This example shows how to use a pre-configured device name
// The names contain the GPIO pin numbers of a specific device
//
#include <bb_captouch.h>
BBCapTouch bbct;

void setup()
{
  int rc;
  Serial.begin(115200);
  delay(3000); // allow time for CDC-Serial to start

// The full list of device names can be found in bb_captouch.h
  rc = bbct.init(TOUCH_WS_AMOLED_18);
  if (rc == CT_SUCCESS) {
      Serial.printf("bb_captouch init success, type = %d\n", bbct.sensorType());
  } else {
    Serial.println("Error initializing bb_captouch");
    while (1) {};
  }
}

void loop()
{
  TOUCHINFO ti;
  while (1) {
    bbct.getSamples(&ti);
    if (ti.count > 0) {
      Serial.printf("x,y = %d,%d\n", ti.x[0], ti.y[0]);
    }
  }
}
