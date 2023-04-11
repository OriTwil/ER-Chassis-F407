# ER-Chassis-F407

ER robot chassis for ROBOCON 2023, ported to STM32F407 MCU.

## Version History
- 2023.4.11: Initial release. Requires further development to finalize remote control operation, improve code reliability and compliance.

## Installation and Usage
- Clone the repository.
- For VS Code users, open the workspace file directly.
- For Keil users, modify CubeMX settings, regenerate the project, copy the Usercode folder and modify main.c (define the clock callback as weak).

## File Description
- Usercode folder, to facilitate porting.

## Robot Hardware Specifications
- The main control board is self-made by the hardware team (Xing).
- Positioning uses a positioning system that is connected to the main control board via an RS232 to TTL module and a serial port.
- The steering wheel uses a VESC electronic speed controller (ESC), with a DJI2006 for steering, and is connected to the CAN bus in parallel.
- The position of the steering wheel is calibrated using three Hall effect sensors, which are connected to the main control board's GPIO.
- Infrared ranging is used for auxiliary positioning, which is connected to the main control board's SPI via a custom-made high-precision analog-to-digital converter module designed by the hardware team.
- A small screen is used for debugging and is plugged into the main control board's pin header via SPI communication.
- Inter-board communication and communication with the host computer: serial port + MAVLINK.
- Remote control: AS69.
- Wireless debugger: can be used as a wireless serial port and for wireless burning, debugging.


