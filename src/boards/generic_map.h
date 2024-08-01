/*
  generic_map.h - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

#if N_ABC_MOTORS > 0
#error "Axis configuration is not supported!"
#endif

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_PN                 2
#define STEP_PORT               port(STEP_PN)
#define X_STEP_PIN              1
#define Y_STEP_PIN              2
#define Z_STEP_PIN              3
#define STEP_OUTMODE            GPIO_MAP

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PN            0
#define DIRECTION_PORT          port(DIRECTION_PN)
#define X_DIRECTION_PIN         11
#define Y_DIRECTION_PIN         20
#define Z_DIRECTION_PIN         22
//#define DIRECTION_OUTMODE GPIO_MAP
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PN      0
#define STEPPERS_ENABLE_PORT    port(STEPPERS_ENABLE_PN)
#define STEPPERS_ENABLE_PIN     10

// Define homing/hard limit switch input pins
// NOTE: All limit bit pins must be on the same port
#define LIMIT_PN                0
#define LIMIT_PORT              port(LIMIT_PN)
#define X_LIMIT_PIN             24
#define Y_LIMIT_PIN             26
#define Z_LIMIT_PIN             29

#define LIMIT_INMODE            GPIO_BITBAND

// Define flood and mist coolant output pins.
#define COOLANT_FLOOD_PN        2
#define COOLANT_FLOOD_PORT      port(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_PIN       4

#define COOLANT_MIST_PN         2
#define COOLANT_MIST_PORT       port(COOLANT_MIST_PN)
#define COOLANT_MIST_PIN        6

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).

#define CONTROL_PN              0
#define CONTROL_PORT            port(CONTROL_PN)
#define RESET_PIN               6
#define FEED_HOLD_PIN           7
#define CYCLE_START_PIN         8
#define CONTROL_INMODE          GPIO_BITBAND

#define AUXINPUT0_PORT          port(CONTROL_PN)
#define AUXINPUT0_PIN           9
#define AUXINPUT1_PN            4
#define AUXINPUT1_PORT          port(AUXINPUT1_PN)
#define AUXINPUT1_PIN           6

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#ifdef SPINDLE_PWM_PIN_2_4
#define SPINDLE_PWM_CHANNEL             PWM1_CH5    // MOSFET3 (P2.4)
#else
#define SPINDLE_PWM_CHANNEL             PWM1_CH6    // BED MOSFET (P2.5)
#endif
#define SPINDLE_PWM_USE_PRIMARY_PIN     false
#define SPINDLE_PWM_USE_SECONDARY_PIN   true
#else
#define AUXOUTPUT0_PN                   2
#define AUXOUTPUT0_PORT                 port(AUXOUTPUT0_PN)
#define AUXOUTPUT0_PIN                  4
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PN            1
#define SPINDLE_DIRECTION_PORT          port(SPINDLE_DIRECTION_PN)
#define SPINDLE_DIRECTION_PIN           19
#else
#define AUXOUTPUT1_PN                   1
#define AUXOUTPUT1_PORT                 port(AUXOUTPUT1_PN)
#define AUXOUTPUT1_PIN                  19
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PN               1
#define SPINDLE_ENABLE_PORT             port(SPINDLE_ENABLE_PN)
#define SPINDLE_ENABLE_PIN              18
#else
#define AUXOUTPUT2_PN                   1
#define AUXOUTPUT2_PORT                 port(AUXOUTPUT2_PN)
#define AUXOUTPUT2_PIN                  18
#endif

#if SDCARD_ENABLE
#define SD_SPI_PORT             0
#define SD_CS_PN                0
#define SD_CS_PORT              port(SD_CS_PN)
#define SD_CS_PIN               16
#endif

/**/
