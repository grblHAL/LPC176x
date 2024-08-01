/*
  ramps_1.6_map.h - driver code for LPC176x processor, pin mappings compatible with Ramps 1.6 board

  NOTE: board must be modified for 3.3V IO before use!

  Part of grblHAL

  Copyright (c) 2019-2024 Terje Io

  Mappings according to Re-ARM for NXP LCP1768

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

// NOTE:
// P0.27, P0.28 are dedicated I2C pins without pull up/down.
// P0.29, P0.30 must have same direction as used for USB operation.

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Ramps 1.6"

// Define step pulse output pins.

#define X_STEP_PN               2
#define X_STEP_PORT             port(X_STEP_PN)
#define X_STEP_PIN              1   // Due Analog Pin 0
#define Y_STEP_PN               2
#define Y_STEP_PORT             port(Y_STEP_PN)
#define Y_STEP_PIN              2   // Due Analog Pin 6
#define Z_STEP_PN               2
#define Z_STEP_PORT             port(Z_STEP_PN)
#define Z_STEP_PIN              3   // Due Digital Pin 46
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PN          0
#define X_DIRECTION_PORT        port(X_DIRECTION_PN)
#define X_DIRECTION_PIN         11  // Due Analog Pin 1
#define Y_DIRECTION_PN          0
#define Y_DIRECTION_PORT        port(Y_DIRECTION_PN)
#define Y_DIRECTION_PIN         20  // Due Analog Pin 7
#define Z_DIRECTION_PN          0
#define Z_DIRECTION_PORT        port(Z_DIRECTION_PN)
#define Z_DIRECTION_PIN         22  // Due Digital Pin 48
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PN             0
#define X_ENABLE_PORT           port(X_ENABLE_PN)
#define X_ENABLE_PIN            10  // Due Digital Pin 38
#define Y_ENABLE_PN             0
#define Y_ENABLE_PORT           port(Y_ENABLE_PN)
#define Y_ENABLE_PIN            19  // Due Analog Pin 2
#define Z_ENABLE_PN             0
#define Z_ENABLE_PORT           port(Z_ENABLE_PN)
#define Z_ENABLE_PIN            21  // Due Analog Pin 8

// Define homing/hard limit switch input pins.
// NOTE: All limit bits (needs to be on same port)
#define X_LIMIT_PN              1
#define X_LIMIT_PORT            port(X_LIMIT_PN)
#define X_LIMIT_PIN             24  // Due Digital Pin 3
#define Y_LIMIT_PN              1
#define Y_LIMIT_PORT            port(Y_LIMIT_PN)
#define Y_LIMIT_PIN             26  // Due Digital Pin 14
#define Z_LIMIT_PN              1
#define Z_LIMIT_PORT            port(Z_LIMIT_PN)
#define Z_LIMIT_PIN             29  // Due Digital Pin 18

// Define max homing/hard limit switch input pins.
#if LIMIT_MAX_ENABLE
#define X_LIMIT_PN_MAX      1
#define X_LIMIT_PORT_MAX    port(X_LIMIT_PN_MAX)
#define X_LIMIT_PIN_MAX     25  // Due Digital Pin 2
#define X_LIMIT_BIT_MAX     (1<<X_LIMIT_PIN_MAX)
#define Y_LIMIT_PN_MAX      1
#define Y_LIMIT_PORT_MAX    port(Y_LIMIT_PN_MAX)
#define Y_LIMIT_PIN_MAX     27  // Due Digital Pin 15
#define Y_LIMIT_BIT_MAX     (1<<Y_LIMIT_PIN_MAX)
#define Z_LIMIT_PN_MAX      1
#define Z_LIMIT_PORT_MAX    port(Z_LIMIT_PN_MAX)
#define Z_LIMIT_PIN_MAX     28  // Due Digital Pin 19
#define Z_LIMIT_BIT_MAX     (1<<Z_LIMIT_PIN_MAX)
#endif

#define LIMITS_POLL_PORT        port(1) // NOTE: Port 1 is not interrupt capable, use polling instead!
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PN              2
#define M3_STEP_PORT            port(M3_STEP_PN)
#define M3_STEP_PIN             0   // Due Digital Pin 26
#define M3_DIRECTION_PN         0
#define M3_DIRECTION_PORT       port(M3_DIRECTION_PN)
#define M3_DIRECTION_PIN        5   // Due Digital Pin 28
#define M3_ENABLE_PN            0
#define M3_ENABLE_PORT          port(M3_ENABLE_PN)
#define M3_ENABLE_PIN           4   // Due Digital Pin 24
#if !LIMIT_MAX_ENABLE
#define M3_LIMIT_PN             1
#define M3_LIMIT_PORT           port(M3_LIMIT_PN)
#define M3_LIMIT_PIN            25  // Due Digital Pin 2
#endif
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PN              2
#define M4_STEP_PORT            port(M4_STEP_PN)
#define M4_STEP_PIN             8   // Due Digital Pin 36
#define M4_DIRECTION_PN         2
#define M4_DIRECTION_PORT       port(M4_DIRECTION_PN)
#define M4_DIRECTION_PIN        13  // Due Digital Pin 34
#define M4_ENABLE_PN            4
#define M4_ENABLE_PORT          port(M4_ENABLE_PN)
#define M4_ENABLE_PIN           29  // Due Digital Pin 30
#if !LIMIT_MAX_ENABLE
#define M4_LIMIT_PN             1
#define M4_LIMIT_PORT           port(M4_LIMIT_PN)
#define M4_LIMIT_PIN            27  // Due Digital Pin 15
#endif
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
#define AUXOUTPUT3_PN                   2
#define AUXOUTPUT3_PORT                 port(AUXOUTPUT3_PN)
#define AUXOUTPUT3_PIN                  4
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PN            1
#define SPINDLE_DIRECTION_PORT          port(SPINDLE_DIRECTION_PN)
#define SPINDLE_DIRECTION_PIN           19
#else
#define AUXOUTPUT4_PN                   1
#define AUXOUTPUT4_PORT                 port(AUXOUTPUT4_PN)
#define AUXOUTPUT4_PIN                  19
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PN               1
#define SPINDLE_ENABLE_PORT             port(SPINDLE_ENABLE_PN)
#define SPINDLE_ENABLE_PIN              18
#else
#define AUXOUTPUT5_PN                   1
#define AUXOUTPUT5_PORT                 port(AUXOUTPUT5_PN)
#define AUXOUTPUT5_PIN                  18
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PN        0
#define COOLANT_FLOOD_PORT      port(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_PIN       26  // Due Analog port 9

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT_PN           0
#define RESET_PORT              port(RESET_PORT_PN)
#define RESET_PIN               27  // DUE Analog Pin 3

#define FEED_HOLD_PN            0
#define FEED_HOLD_PORT          port(FEED_HOLD_PN)
#define FEED_HOLD_PIN           28  // DUE Analog Pin 4

#define CYCLE_START_PN          2
#define CYCLE_START_PORT        port(CYCLE_START_PN)
#define CYCLE_START_PIN         6   // DUE Analog Pin 5

#define CONTROL_INMODE          GPIO_BITBAND

#define AUXINPUT0_PN            0
#define AUXINPUT0_PORT          port(AUXINPUT0_PN)
#define AUXINPUT0_PIN           15
#define AUXINPUT1_PN            0
#define AUXINPUT1_PORT          port(AUXINPUT1_PN)
#define AUXINPUT1_PIN           17
#define AUXINPUT2_PN            4
#define AUXINPUT2_PORT          port(AUXINPUT2_PN)
#define AUXINPUT2_PIN           6

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT2_PORT
#define PROBE_PIN               AUXINPUT2_PIN
#endif

#define SD_SPI_PORT             1
#define SD_CS_PN                0
#define SD_CS_PORT              port(SD_CS_PN)
#define SD_CS_PIN               6

#define AUXOUTPUT0_PN           1
#define AUXOUTPUT0_PORT         port(AUXOUTPUT0_PN)
#define AUXOUTPUT0_PIN          23
#define AUXOUTPUT1_PN           0
#define AUXOUTPUT1_PORT         port(AUXOUTPUT1_PN)
#define AUXOUTPUT1_PIN          18
#define AUXOUTPUT2_PN           1
#define AUXOUTPUT2_PORT         port(AUXOUTPUT2_PN)
#define AUXOUTPUT2_PIN          31

/**/
