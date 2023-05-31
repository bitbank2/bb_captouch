BitBank Capacitive Touch Sensor Library<br>
---------------------------------------
Copyright (c) 2023 BitBank Software, Inc.<br>
Written by Larry Bank<br>
bitbank@pobox.com<br>
<br>
GOODiX and FocalTech make many different capacitive touch controllers and this library supports the GT911 and FT6x36 ones in a generic way. Each has different capabilities and usually come pre-programmed for the specific pixel width and height of the target application. A feature supported by this library that may not be present in the device you're using is the touch area and pressure values. Some of their controllers also have built-in gesture detection. The common features of the controllers is that they will generate an interrupt signal when a touch event is occurring. This library allows you to request the latest touch information and it returns the number of active touch points (0-5) along with the coordinates (and pressure/area of each if available). The sensor type and address is auto-detected when calling the init() method. The only info that must be correctly supplied to the library are the GPIO pins used for the SDA/SCL/INT/RESET signals. Once initialized, repeatedly call getSamples() to test for and read any touch samples available.<br>

