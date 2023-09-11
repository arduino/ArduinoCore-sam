# Arduino Core for SAM3 and SAM4 CPU

This repository contains the source code and configuration files of the Arduino Core for Atmel's SAM3X processor (used on the [Arduino Due](https://www.arduino.cc/en/Main/ArduinoBoardDue) board).

## Installation on Arduino IDE

This core is available as a package in the Arduino IDE cores manager.
Just open the "Boards Manager" and install the package called:

 * **Arduino SAM Boards (32-bit ARM Cortex-M3)**

## Installation on VSCode

Use Platformio extension with atmelsam framework

```
[env:due]
platform = atmelsam
framework = arduino
board = due
    
```

Development SAM4 Version

```
[env:sam4s4a_SAM4]
platform = atmelsam
framework = arduino
board = motionbow
platform_packages =
    framework-arduino-sam@https://github.com/chepo92/ArduinoCore-sam.git#dev-SAM4SCore
    toolchain-gccarmnoneeabi @ 1.40804.0

```

## Support

There is a dedicated section of the Arduino Forum for general discussion and project assistance:

http://forum.arduino.cc/index.php?board=87.0

## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/arduino/ArduinoCore-sam/issues

Before posting a new issue, please check if the same problem has been already reported by someone else to avoid duplicates.

## Contributions

Contributions are always welcome.
The preferred way to receive code contribution is by submitting a Pull Request on github.

