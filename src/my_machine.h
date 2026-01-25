/*
  my_machine.h - configuration for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
//#define BOARD_BTT_SKR_13
//#define BOARD_BTT_SKR_14_TURBO
//#define BOARD_BTT_SKR_E3_TURBO  // With onboard Trinamic TNMC2209 drivers. NOTE: not verified!
//#define BOARD_MKS_SBASE_13
//#define BOARD_RAMPS_16
//#define BOARD_SMOOTHIEBOARD
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC           1 // Comment out to use UART communication.
#endif
//#define BLUETOOTH_ENABLE         2 // Set to 2 for HC-05 module. Requires and claims one auxillary input pin.
// Spindle selection:
// Up to four specific spindle drivers can be instantiated at a time
// depending on N_SPINDLE and N_SYS_SPINDLE definitions in grbl/config.h.
// If none are specified the default PWM spindle is instantiated.
// Spindle definitions can be found in grbl/spindle_control.h.
// More here https://github.com/grblHAL/Plugins_spindle
//#define SPINDLE0_ENABLE          SPINDLE_PWM0
//#define SPINDLE1_ENABLE          SPINDLE_HUANYANG1
//#define SPINDLE2_ENABLE          SPINDLE_PWM0_CLONE
//#define SPINDLE3_ENABLE          SPINDLE_NONE
// **********************
//#define MODBUS_ENABLE            1 // Set to 1 for auto direction, 2 for direction signal on auxillary output pin.
//#define SDCARD_ENABLE            1 // Run gcode programs from SD card. Set to 2 to enable YModem upload.
//#define MPG_ENABLE               2 // Enable MPG interface. Requires a serial port and means to switch between normal and MPG mode.
                                     // 1: Mode switching is by handshake pin.
                                     // 2: Mode switching is by the CMD_MPG_MODE_TOGGLE command character.
//#define KEYPAD_ENABLE            2 // 1: uses a I2C keypad for input.
                                     // 2: uses a serial port for input. If MPG_ENABLE is set > 0 the serial stream is shared with the MPG.
//#define LASER_COOLANT_ENABLE     1 // Laser coolant plugin. To be completed.
//#define LB_CLUSTERS_ENABLE       1 // LaserBurn cluster support.
//#define TRINAMIC_ENABLE      5160 // Trinamic TMC5160 stepper driver support.
//#define TRINAMIC_ENABLE      2209 // Trinamic TMC2209 stepper driver support.
//#define TRINAMIC_ENABLE      2660 // Trinamic TMC2660 stepper driver support.
//#define TRINAMIC_R_SENSE      110 // R sense resistance in milliohms, 2130 and 2209 default is 110, 5160 is 75.
//#define TRINAMIC_ENABLE      2240 // Trinamic TMC2240 stepper driver support.
//#define TRINAMIC_R_REF         12 // R ref resistance in kiloohms, used for 2240 - default value is 12.
//#define EEPROM_ENABLE          16 // I2C EEPROM/FRAM support. Set to 16 for 2K, 32 for 4K, 64 for 8K, 128 for 16K and 256 for 16K capacity.
//#define EEPROM_IS_FRAM          1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
#define ESTOP_ENABLE             0 // When enabled only real-time report requests will be executed when the reset pin is asserted.
                                   // Note: if commented out the default setting is determined from COMPATIBILITY_LEVEL.
// Optional control signals:
// These will be assigned to aux input pins. Use the $pins command to check which pins are assigned.
// NOTE: If not enough pins are available assignment will silently fail.
//#define PROBE_ENABLE            0 // Default enabled, uncomment to disable probe input or uncomment and set to 2 to enable relay switched probes.
//#define PROBE2_ENABLE           1 // Enable second regular probe input, depending on the board the input assigned may be predefined.
//#define TOOLSETTER_ENABLE       1 // Enable toolsetter input, depending on the board the input assigned may be predefined.
//#define SAFETY_DOOR_ENABLE      1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1
//#define PROBE_DISCONNECT_ENABLE 1
//#define STOP_DISABLE_ENABLE     1
//#define BLOCK_DELETE_ENABLE     1
//#define SINGLE_BLOCK_ENABLE     1
//#define LIMIT_MAX_ENABLE        1 // Uncomment to enable max limit input pins (when available)
//#define LIMITS_OVERRIDE_ENABLE  1

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
