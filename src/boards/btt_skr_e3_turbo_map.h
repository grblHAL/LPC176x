/*
  btt_skr_e3_turbo_map.h - driver code for LPC176x processor, pin mappings compatible with BTT SKR E3 Turbo board

  Part of grblHAL

  Copyright (c) 2021-2024 Dimitris Zervas, Terje Io

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

#define BOARD_NAME "BTT SKR E3 Turbo"
#ifdef TRINAMIC_ENABLE
#undef TRINAMIC_ENABLE
#endif
#define TRINAMIC_ENABLE 2209
#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_UART_ENABLE 1
#define HAS_BOARD_INIT

// Define step pulse output pins.
#define X_STEP_PN               1
#define X_STEP_PORT             port(X_STEP_PN)
#define X_STEP_PIN              4
#define Y_STEP_PN               1
#define Y_STEP_PORT             port(Y_STEP_PN)
#define Y_STEP_PIN              14
#define Z_STEP_PN               4
#define Z_STEP_PORT             port(Z_STEP_PN)
#define Z_STEP_PIN              29
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PN          1
#define X_DIRECTION_PORT        port(X_DIRECTION_PN)
#define X_DIRECTION_PIN         8
#define Y_DIRECTION_PN          1
#define Y_DIRECTION_PORT        port(Y_DIRECTION_PN)
#define Y_DIRECTION_PIN         15
#define Z_DIRECTION_PN          4
#define Z_DIRECTION_PORT        port(Z_DIRECTION_PN)
#define Z_DIRECTION_PIN         28
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PN             1
#define X_ENABLE_PORT           port(X_ENABLE_PN)
#define X_ENABLE_PIN            0
#define Y_ENABLE_PN             1
#define Y_ENABLE_PORT           port(Y_ENABLE_PN)
#define Y_ENABLE_PIN            9
#define Z_ENABLE_PN             1
#define Z_ENABLE_PORT           port(Z_ENABLE_PN)
#define Z_ENABLE_PIN            16

// Define homing/hard limit switch input pins.
// NOTE: All limit bits (needs to be on same port)
#define LIMIT_PN                1
#define LIMIT_PORT              port(LIMIT_PN)
#define X_LIMIT_PIN             29
#define Y_LIMIT_PIN             28
#define Z_LIMIT_PIN             27
#define LIMITS_POLL_PORT        port(1) // NOTE: Port 1 is not interrupt capable, use polling instead!
#define LIMIT_INMODE            GPIO_BITBAND

// Define standby pins
#define X_STANDBY
#define X_STANDBY_PORT          port(3)
#define X_STANDBY_PIN           26
#define Y_STANDBY
#define Y_STANDBY_PORT          port(3)
#define Y_STANDBY_PIN           25
#define Z_STANDBY
#define Z_STANDBY_PORT          port(1)
#define Z_STANDBY_PIN           18

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PN              2
#define M3_STEP_PORT            port(M3_STEP_PN)
#define M3_STEP_PIN             6
#define M3_DIRECTION_PN         2
#define M3_DIRECTION_PORT       port(M3_DIRECTION_PN)
#define M3_DIRECTION_PIN        7
#define M3_LIMIT_PORT           port(LIMIT_PN)
#define M3_LIMIT_PIN            26
#define M3_ENABLE_PN            0
#define M3_ENABLE_PORT          port(M3_ENABLE_PN)
#define M3_ENABLE_PIN           4
#define M3_STANDBY
#define M3_STANDBY_PORT         port(1)
#define M3_STANDBY_PIN          19
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PN              2
#define M4_STEP_PORT            port(M4_STEP_PN)
#define M4_STEP_PIN             11
#define M4_DIRECTION_PN         2
#define M4_DIRECTION_PORT       port(M4_DIRECTION_PN)
#define M4_DIRECTION_PIN        12
#define M4_LIMIT_PORT           port(LIMIT_PN)
#define M4_LIMIT_PIN            25
#define M4_ENABLE_PN            0
#define M4_ENABLE_PORT          port(M4_ENABLE_PN)
#define M4_ENABLE_PIN           21
#define M4_STANDBY
#define M4_STANDBY_PORT         port(2)
#define M4_STANDBY_PIN          13
#endif

// Define auxiliary output pins
#define AUXOUTPUT0_PN           2                   // Spindle PWM
#define AUXOUTPUT0_PORT         port(AUXOUTPUT0_PN)
#define AUXOUTPUT0_PIN          4
#define AUXOUTPUT1_PN           1                   // Spindle direction
#define AUXOUTPUT1_PORT         port(AUXOUTPUT1_PN)
#define AUXOUTPUT1_PIN          20
#define AUXOUTPUT2_PN           0                   // Spindle enable
#define AUXOUTPUT2_PORT         port(AUXOUTPUT2_PN)
#define AUXOUTPUT2_PIN          16
#define AUXOUTPUT3_PN           0                   // Coolant flood
#define AUXOUTPUT3_PORT         port(AUXOUTPUT3_PN)
#define AUXOUTPUT3_PIN          19
#define AUXOUTPUT4_PN           1                   // Coolant mist
#define AUXOUTPUT4_PORT         port(AUXOUTPUT4_PN)
#define AUXOUTPUT4_PIN          30

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_CHANNEL             PWM1_CH5    // MOSFET3 (P2.4)
#define SPINDLE_PWM_USE_PRIMARY_PIN     true
#define SPINDLE_PWM_USE_SECONDARY_PIN   false
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

// Define user-control CONTROLs (reset, feed hold, cycle start) input pins.
#define RESET_PORT_PN           0
#define RESET_PORT              port(RESET_PORT_PN)
#define RESET_PIN               18

#define FEED_HOLD_PN            0
#define FEED_HOLD_PORT          port(FEED_HOLD_PN)
#define FEED_HOLD_PIN           17

#define CYCLE_START_PN          0
#define CYCLE_START_PORT        port(CYCLE_START_PN)
#define CYCLE_START_PIN         15

#define CONTROL_INMODE          GPIO_BITBAND

#define AUXINPUT0_PORT          port(2) // EXP1-1
#define AUXINPUT0_PIN           8
#define AUXINPUT1_PORT          port(1) // Z-PROBE - Probe
#define AUXINPUT1_PIN           22
#define AUXINPUT2_PORT          port(0) // EXP1-6 - Reset/EStop
#define AUXINPUT2_PIN           18
#define AUXINPUT3_PORT          port(0) // EXP1-2 - Feed hold
#define AUXINPUT3_PIN           16
#define AUXINPUT4_PORT          port(0) // EXP1-6 - Cycle start
#define AUXINPUT4_PIN           15

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
#define SD_CS_PIN               6
#endif

//#define TMC_STEALTHCHOP         0 // CoolStep
#define MOTOR_UARTX_PN          1
#define MOTOR_UARTX_PORT        port(MOTOR_UARTX_PN)
#define MOTOR_UARTX_PIN         1
#define MOTOR_UARTY_PN          1
#define MOTOR_UARTY_PORT        port(MOTOR_UARTY_PN)
#define MOTOR_UARTY_PIN         10
#define MOTOR_UARTZ_PN          1
#define MOTOR_UARTZ_PORT        port(MOTOR_UARTZ_PN)
#define MOTOR_UARTZ_PIN         17
#ifdef M3_AVAILABLE
#define MOTOR_UARTM3_PN         0
#define MOTOR_UARTM3_PORT       port(MOTOR_UARTM3_PN)
#define MOTOR_UARTM3_PIN        5
#endif
#ifdef M4_AVAILABLE
#define MOTOR_UARTM4_PN         0
#define MOTOR_UARTM4_PORT       port(MOTOR_UARTM4_PN)
#define MOTOR_UARTM4_PIN        22
#endif

/**/
