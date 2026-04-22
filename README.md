
# NuMicro ISP Tool

The NuMicro ISP Tool is an open-source utility for programming and configuring Nuvoton microcontrollers via In-System Programming (ISP). It supports multiple interfaces (USB HID, UART, SPI, I²C, RS485, CAN, LIN, Wi-Fi, BLE) and communicates with the ISP application running in LDROM on the target device.

## Features

- Cross-interface ISP programming (USB HID, UART, SPI, I²C, RS485, CAN, etc.)
- Supports Nu-Link2-Pro/Nu-Link3-Pro ISP-Bridge mode
- GUI for easy device configuration and programming
- Chip database for automatic device recognition
- Extensible with new chips and protocols

## Directory Structure

- `NuvoISP/` — Main application source code (C++)
	- `DataBase/` — Chip definitions and device database
	- `DialogChipSetting/` — Per-chip configuration dialogs
	- `DialogConfiguration/` — Device configuration logic
	- `Interface/` — Interface drivers (HID, UART, etc.)
	- `res/` — Application resources (icons, bitmaps)
	- `ThirdParty/hidapi/` — HID API library for USB communication
- `Documents/` — User manuals and programming guides
	- `NuMicro_ISP_Programming_User_Manual/`
	- `NuMicro_ISP_Programming_Guide/`

## Getting Started

### Prerequisites
- Microsoft Visual Studio 2019 or later
- Windows OS

### Build Instructions
1. Open `NuvoISP/NuvoISP_VS2019.sln` in Visual Studio.
2. Restore any required packages (if prompted).
3. Build the solution (Release or Debug).
4. Run the generated executable.

### Usage
1. Connect your Nu-Link or ISP-Bridge to the target board.
2. Launch the ISP Tool application.
3. Select the interface and target chip.
4. Follow the on-screen instructions to program or configure the device.

## Documentation

- [NuMicro ISP Programming User Manual](Documents/NuMicro_ISP_Programming_User_Manual/NuMicro_ISP_Programming_User_Manual.md)
- [NuMicro ISP Programming Guide](Documents/NuMicro_ISP_Programming_Guide/NuMicro_ISP_Programming_Guide.md)

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

