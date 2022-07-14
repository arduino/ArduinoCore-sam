# Enhanced Arduino Core for SAM3X CPU

This repository is based from the [Arduio Sam SDK for cortex-m3](https://github.com/arduino/ArduinoCore-sam). The sdk was modified to include the following features which are not supported by the main repository:

- Attachment of a callback function to the IRQ handler of USART/UART peripherals. This comes handy when you need to process data in real-time.
- Addition of USART2 as Serial4.
- Updated GCC to version 11.2.1.
- Using by default C++20 and C17

## Installation

Add the following url to your Arduino package manager

https://raw.githubusercontent.com/vChavezB/ArduinoCore-sam/master/package.json

---
The rest of this readme has been kept unmodified and is as-is from the original repo.

---

This repository contains the source code and configuration files of the Arduino Core for Atmel's SAM3X processor (used on the [Arduino Due](https://www.arduino.cc/en/Main/ArduinoBoardDue) board).

## Installation on Arduino IDE

This core is available as a package in the Arduino IDE cores manager.
Just open the "Boards Manager" and install the package called:

 * **Arduino SAM Boards (32-bit ARM Cortex-M3)**

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
