# ER-Chassis-F407

ER robot chassis for ROBOCON 2023, ported to STM32F407 MCU.

## "Elephent Robot"

## Version History
- 2023.4.11: Initial release. Requires further development to finalize remote control operation, improve code reliability and compliance.

## Installation and Usage
- Clone the repository.
- For VS Code users, open the workspace file directly.
- For Keil users, modify CubeMX settings, regenerate the project, copy the Usercode folder and modify main.c (define the clock callback as weak).

## File Description
- Usercode folder, to facilitate porting.

## Robot Hardware Specifications
- The main control board is self-made by the hardware team (Xing) `Stm32F407`.
- Positioning uses a positioning system `OPS-9`that is connected to the main control board via an `RS232 to TTL` module and a serial port.
- The steering wheel uses a `VESC` electronic speed controller (ESC), with a `DJI2006` for steering, and is connected to the CAN bus in parallel.
- The encoder uses an `AMT102-V` incremental capacitive modular encoder.
- The position of the steering wheel is calibrated using three Hall effect sensors`NJK-5001A`, which are connected to the main control board's GPIO.
- Infrared ranging `DT35` is used for auxiliary positioning, which is connected to the main control board's SPI via a custom-made high-precision analog-to-digital converter module designed by the hardware team.
- A `OLED` is used for debugging and is plugged into the main control board's pin header via SPI communication.
- Inter-board communication and communication with the host computer: serial port + MAVLINK.
- Remote control: `AS69`.
- `Wireless debugger`: can be used as a wireless serial port and for wireless burning, debugging.

## Cable Routing Considerations
### Cable Routing Experience
- For multiple wires or long cables, consider the cable's position during mechanical design (set grooves, through-holes, etc.).
- Make use of adapter boards to reduce the number of wires (e.g., power and CAN signal wires). Use adapter boards instead of multiple wires to split the signal at the required locations.
- Route cables to avoid interference with mechanical structures.
- Leave redundant lengths at corners and stretches.
- Ensure that cables are not exposed.
- Avoid reciprocal movement of cables during design.
- Keep signal cables away from sources of interference.
- Secure non-moving cables firmly.
- Use snake skin mesh or spiral wrap to protect unshielded wires with a diameter less than 22AWG (0.64mm).
- For rotating spaces: (1) use conductive slip rings (360 degrees), (2) route cables through hollow axles, and (3) reserve redundant length and fix both ends at the rotating axis.
- For translation structures, use drag chains and consider space and fixing during design.
- For CAN cable routing: CAN network
- Do not use air docking connectors, extend the wires as much as possible and secure them.
- For complex driving mechanisms, reuse power cables (common ground and wires with the same voltage).
- For metal and carbon fiber cars, be cautious about grounding the shell to prevent shorts caused by accidental contact with power cables.
- For main power cables, choose multi-strand wires for redundancy or thicker wires.
- Consider interference resistance, use double-twisted method and short cable routing for signal cables (serial port, CAN), and use shielded wires for longer ones.

### Common Cable Routing Consumables
- Zip ties: used to fix cables, cut with diagonal pliers, generally black.
<image src="https://github.com/OriTwil/ER-Chassis-F407/blob/main/picture/zip.jpg" width="300" height="210">

- Snake skin mesh: used to protect cables, both ends of the mesh need to be fixed with zip ties.
<image src="https://github.com/OriTwil/ER-Chassis-F407/blob/main/picture/mesh.png" width="300" height="210">

- Spiral wrap.
<image src="https://github.com/OriTwil/ER-Chassis-F407/blob/main/picture/protect.jpg" width="300" height="210">

- Tape: insulation tape has low adhesion, while fabric tape has higher structural strength.
<image src="https://github.com/OriTwil/ER-Chassis-F407/blob/main/picture/buji.jpg" width="300" height="210">
<image src="https://github.com/OriTwil/ER-Chassis-F407/blob/main/picture/jueyuanjiaodai.jpg" width="300" height="210">

- Hot melt glue and clay: used to prevent joint detachment, applied before the race.
- Drag chains.
<image src="https://github.com/OriTwil/ER-Chassis-F407/blob/main/picture/tuolian.jpg" width="300" height="210">

### Marking
- Indicate the location of sensors on the sensor control board/main control board (e.g., upper left, lower left; 1-①, 1-②, etc.).
- Use the same color of marking tape or label paper at both ends of the same wire (used for one sensor). 



