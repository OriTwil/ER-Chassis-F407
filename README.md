# ER-Chassis-F407

ER robot chassis for ROBOCON 2023, ported to STM32F407 MCU.

## Elephent Robot

## File Description
- Usercode folder, to facilitate porting.
- <image src="https://github.com/OriTwil/ER-Chassis-F407/blob/main/picture/New_code_structure.png" width="1000">
  
## Dynamics

## Version History
- 2023.4.11: Initial release. Requires further development to finalize remote control operation, improve code reliability and compliance.
- 2023.5.27: The basic completion of the entire code, only the remote controller and operation design part are left, and parameter adjustment is needed.

## Installation and Usage
- Clone the repository.
- For VS Code users, open the workspace file directly.
- For Keil users, modify CubeMX settings, regenerate the project, copy the Usercode folder and modify main.c (define the clock callback as weak).

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

## EIDE 工程说明

### 目录结构

- CubeMX: 用于存放 CubeMX 的 ioc 文件和 CubeMX 生成的所有文件
- UserCode: 用于存放用户代码文件
- .eide: EIDE 相关的配置文件
  - debug.files.options.yml: 可以给不同的文件指定不同的编译选项

### 编译器选项说明

#### -Wextra

开启额外的 warning，这样会打印更多的 warning，更容易找到 bug

#### 选项：One ELF Section per Function 和 One ELF Section per Data

勾上这两个选项分别等价于添加 `-ffunction-sections` 和 `-fdata-sections` 编译选项。

`-ffunction-sections` 和 `-fdata-sections` 用于告诉编译器将每个函数和数据放在单独的段中，以便在链接时可以更好地优化代码大小。

#### -fno-exceptions

-fno-exceptions 是 GCC 编译器的一个选项，它表示禁用 C++ 异常机制。这个选项会让编译器生成更小、更快的代码，但是会使得程序无法使用 C++ 异常机制。如果你的程序中没有使用 C++ 异常机制，那么可以使用这个选项来优化你的程序。

#### -pipe

-pipe 选项告诉 GCC 使用管道而不是临时文件来在编译的各个阶段之间进行通信。这可以加快编译速度，特别是在 I/O 较慢的系统上。

#### -fno-rtti

-fno-rtti 选项告诉 GCC 不要生成与运行时类型信息有关的代码。这可以减小程序的大小并提高编译速度，但是会使得程序无法使用 RTTI（Run-Time Type Identification）功能。

### 链接器选项说明

#### 选项：Remove Unused Input Sections

勾上这个选项分别等价于添加 `-Wl,--gc-sections` 编译选项。

其中，`-Wl` 是 GCC 的编译选项，用于将后面的参数传递给链接器。

`--gc-sections`用于告诉链接器在链接时删除未使用的段。

因此，配合`-ffunction-sections` 和 `-fdata-sections` 选项可以实现删除未使用的函数和数据（如未使用的全局变量）。

#### --specs=nosys.specs

用于告诉链接器不使用系统调用（system calls）

标准 C 库中的一些功能（比如控制台输入输出、文件读写、获取时间）需要调用操作系统（如 Windows, Linux）提供的函数，这些函数称为系统调用（system calls）。

但是 STM32 没有操作系统，所以也就没有这些函数，于是链接时就会产生找不到这些函数的错误。加上 `--specs=nosys.specs` 之后，链接器就会自己弱定义这些函数，于是就不会产生上述错误了。

链接器自己定义的这些函数不实现任何功能，只是用于去除链接错误的。（所以如果你此时调用`printf`，并不会有任何东西打印出来）

> 当然，你也可以自己实现一些系统调用。例如，实现 `_write()` 和 `_read()` 将 `printf` 重定向到串口。  
> 链接器会优先链接你定义的这些函数。  
> 如果你将所有的系统调用都定义了，也可以不加 `--specs=nosys.specs`，否则就必须加。

#### --specs=nano.specs

用于告诉链接器使用 Nano libc。Nano libc 是一个轻量级的 C 库，它的目标是减小可执行文件的大小。使用 Nano libc 可以减小可执行文件的大小，但是会牺牲一些功能。

默认 Nano libc 关闭了 printf 和 scanf 浮点数的功能。如果要使用该功能，需要在链接器选项中添加 `-u _printf_float` 和 `-u _scanf_float`

> 不添加这一条则会使用标准 C 库。标准 C 库默认支持 printf 和 scanf 浮点数，因此使用标准 C 库时不用添加`-u _printf_float` 和 `-u _scanf_float`

### 链接库

使用 `-lLibraryName` 链接名称为 `LibraryName` 的库。

如：`-lm -lstdc++`

- m：数学库，提供了 math.h 中的各种数学函数。（有时候编译器会默认帮你链接这个库）
- stdc++: C++ 库，提供了许多 C++ 中的函数。

> 编译时使用 `gcc` 命令会默认链接 C 库，不会链接 C++ 库；  
> 编译时使用 `g++` 命令会默认链接 C++ 库；  
> 所以通常情况下使用 `gcc` 编译 .c 文件，使用 `g++` 编译 .cpp 文件  
> 但 EIDE 编译 .cpp 文件时也使用 `gcc` 命令，所以必须指定要链接 C++ 库
  

## Cable Routing Considerations
#### Cable Routing Experience
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

#### Common Cable Routing Consumables
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

#### Marking
- Indicate the location of sensors on the sensor control board/main control board (e.g., upper left, lower left; 1-①, 1-②, etc.).
- Use the same color of marking tape or label paper at both ends of the same wire (used for one sensor). 



