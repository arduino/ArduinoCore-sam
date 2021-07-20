To setup the project and upload the Arduino usbserial application firmware to an ATMEGA16U2:
1. unpack the source into LUFA's Projects directory (LUFA version 100807, http://www.fourwalledcubicle.com/LUFA.php)
2. set ARDUINO_MODEL_PID in the makefile as appropriate, default is Arduino Due
3. do "make clean; make" (the makefile is for winavr)
4. do the connections as in https://www.arduino.cc/en/Hacking/Upgrading16U2Due
5. upload Arduino-usbserial.hex by a proper method with a proper programmer (Arduino as ISP, USBASP, etc.)
6. disconnect the programmer
Test by uploading a new Arduino sketch from the Arduino IDE.
