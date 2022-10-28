/*
  my_machine.h - configuration for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2020-2022 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
//#define SMOOTHIEBOARD
//#define BOARD_RAMPS_16
//#define BOARD_BTT_SKR_13
//#define BOARD_BTT_SKR_14_TURBO
//#define BOARD_BTT_SKR_E3_TURBO  // With onboard Trinamic TNMC2209 drivers. NOTE: not verified!
//#define BOARD_MKS_SBASE_13
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC     1 // Comment out to use UART communication.
#endif
//#define SAFETY_DOOR_ENABLE 1 // Enable safety door input.
//#define SDCARD_ENABLE      1 // Run gcode programs from SD card, requires sdcard plugin.
//#define MPG_ENABLE         1 // Enable MPG interface. Requires serial port and one handshake pin.
//#define TRINAMIC_ENABLE 2130 // Uncomment to enable Trinamic TMC2130 driver support. NOTE: Experimental for now, currently for SKR 1.x boards only
//#define TRINAMIC_ENABLE 2209 // Uncomment to enable Trinamic TMC2209 driver support. NOTE: Experimental for now, currently for SKR 1.x boards only
//#define TRINAMIC_ENABLE 5160 // Uncomment to enable Trinamic TMC5160 driver support. NOTE: Experimental for now, currently for SKR E3 Turbo board only
//#define LIMIT_MAX_ENABLE   1 // Uncomment to enable max limit input pins (when available)
//#define EEPROM_ENABLE      1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 (64 byte pages) or 3 (32 byte pages) for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.

// If the selected board map supports more than three motors ganging and/or auto-squaring
// of axes can be enabled here.
//#define X_GANGED            1
//#define X_AUTO_SQUARE       1
//#define Y_GANGED            1
//#define Y_AUTO_SQUARE       1
//#define Z_GANGED            1
//#define Z_AUTO_SQUARE       1
// For ganged axes the limit switch input (if available) can be configured to act as a max travel limit switch.
// NOTE: If board map already has max limit inputs defined this configuration will be ignored.
//#define X_GANGED_LIM_MAX    1
//#define Y_GANGED_LIM_MAX    1
//#define Z_GANGED_LIM_MAX    1

/*EOF*/
