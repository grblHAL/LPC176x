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

// Define auxiliary output pins
#define AUXOUTPUT0_PN           2                   // Spindle PWM
#define AUXOUTPUT0_PORT         port(AUXOUTPUT0_PN)
#define AUXOUTPUT0_PIN          4
#define AUXOUTPUT1_PN           1                   // Spindle direction
#define AUXOUTPUT1_PORT         port(AUXOUTPUT1_PN)
#define AUXOUTPUT1_PIN          19
#define AUXOUTPUT2_PN           1                   // Spindle enable
#define AUXOUTPUT2_PORT         port(AUXOUTPUT2_PN)
#define AUXOUTPUT2_PIN          18
#define AUXOUTPUT3_PN           2                   // Coolant flood
#define AUXOUTPUT3_PORT         port(AUXOUTPUT3_PN)
#define AUXOUTPUT3_PIN          4
#define AUXOUTPUT4_PN           2                   // Coolant mist
#define AUXOUTPUT4_PORT         port(AUXOUTPUT4_PN)
#define AUXOUTPUT4_PIN          6

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#ifdef SPINDLE_PWM_PIN_2_4
#define SPINDLE_PWM_CHANNEL             PWM1_CH5    // MOSFET3 (P2.4)
#else
#define SPINDLE_PWM_CHANNEL             PWM1_CH6    // BED MOSFET (P2.5)
#endif
#define SPINDLE_PWM_USE_PRIMARY_PIN     false
#define SPINDLE_PWM_USE_SECONDARY_PIN   true
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT          port(0)
#define AUXINPUT0_PIN           9
#define AUXINPUT1_PORT          port(4) // Probe
#define AUXINPUT1_PIN           6
#define AUXINPUT2_PORT          port(0) // Reset/EStop
#define AUXINPUT2_PIN           6
#define AUXINPUT3_PORT          port(0) // Feed hold
#define AUXINPUT3_PIN           7
#define AUXINPUT4_PORT          port(0) // Cycle start
#define AUXINPUT4_PIN           8

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT2_PORT
#define RESET_PIN               AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT3_PORT
#define FEED_HOLD_PIN           AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT4_PORT
#define CYCLE_START_PIN         AUXINPUT4_PIN
#endif

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

#if SDCARD_ENABLE
#define SD_SPI_PORT             0
#define SD_CS_PN                0
#define SD_CS_PORT              port(SD_CS_PN)
#define SD_CS_PIN               16
#endif

/**/
