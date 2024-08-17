/*
  btt_skr_1.3_map.h - driver code for LPC176x processor, pin mappings compatible with BTT SKR v1.3 board

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

// NOTE:
// P0.27, P0.28 are dedicated I2C pins without pull up/down.
// P0.29, P0.30 must have same direction as used for USB operation.

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "BTT SKR V1.3"
#define BOARD_URL "https://github.com/bigtreetech/BIGTREETECH-SKR-V1.3"

#if TRINAMIC_ENABLE
#define HAS_BOARD_INIT
void board_init (void);
#endif

#if EEPROM_ENABLE
#error "This board cannot be used with current EEPROM code!"
#endif

#if I2C_ENABLE
#error "This board does not support I2C!"
#endif

// Define step pulse output pins.
#define X_STEP_PN               2
#define X_STEP_PORT             port(X_STEP_PN)
#define X_STEP_PIN              2
#define Y_STEP_PN               0
#define Y_STEP_PORT             port(Y_STEP_PN)
#define Y_STEP_PIN              19
#define Z_STEP_PN               0
#define Z_STEP_PORT             port(Z_STEP_PN)
#define Z_STEP_PIN              22
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PN          2
#define X_DIRECTION_PORT        port(X_DIRECTION_PN)
#define X_DIRECTION_PIN         6
#define Y_DIRECTION_PN          0
#define Y_DIRECTION_PORT        port(Y_DIRECTION_PN)
#define Y_DIRECTION_PIN         20
#define Z_DIRECTION_PN          2
#define Z_DIRECTION_PORT        port(Z_DIRECTION_PN)
#define Z_DIRECTION_PIN         11
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PN             2
#define X_ENABLE_PORT           port(X_ENABLE_PN)
#define X_ENABLE_PIN            1
#define Y_ENABLE_PN             2
#define Y_ENABLE_PORT           port(Y_ENABLE_PN)
#define Y_ENABLE_PIN            8
#define Z_ENABLE_PN             0
#define Z_ENABLE_PORT           port(Z_ENABLE_PN)
#define Z_ENABLE_PIN            21

// Define homing/hard limit switch input pins.
// NOTE: All limit bits (needs to be on same port)
#define X_LIMIT_PN              1
#define X_LIMIT_PORT            port(X_LIMIT_PN)
#define X_LIMIT_PIN             29
#define Y_LIMIT_PN              1
#define Y_LIMIT_PORT            port(Y_LIMIT_PN)
#define Y_LIMIT_PIN             27
#define Z_LIMIT_PN              1
#define Z_LIMIT_PORT            port(Z_LIMIT_PN)
#define Z_LIMIT_PIN             25

// Define max homing/hard limit switch input pins.
#if LIMIT_MAX_ENABLE
#define X_LIMIT_PN_MAX      1
#define X_LIMIT_PORT_MAX    port(X_LIMIT_PN_MAX)
#define X_LIMIT_PIN_MAX     28
#define X_LIMIT_BIT_MAX     (1<<X_LIMIT_PIN_MAX)
#define Y_LIMIT_PN_MAX      1
#define Y_LIMIT_PORT_MAX    port(Y_LIMIT_PN_MAX)
#define Y_LIMIT_PIN_MAX     26
#define Y_LIMIT_BIT_MAX     (1<<Y_LIMIT_PIN_MAX)
#define Z_LIMIT_PN_MAX      1
#define Z_LIMIT_PORT_MAX    port(Z_LIMIT_PN_MAX)
#define Z_LIMIT_PIN_MAX     24
#define Z_LIMIT_BIT_MAX     (1<<Z_LIMIT_PIN_MAX)
#endif

#define LIMITS_POLL_PORT        port(1) // NOTE: Port 1 is not interrupt capable, use polling instead!
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PN              2
#define M3_STEP_PORT            port(M3_STEP_PN)
#define M3_STEP_PIN             13
#define M3_DIRECTION_PN         0
#define M3_DIRECTION_PORT       port(M3_DIRECTION_PN)
#define M3_DIRECTION_PIN        11
#define M3_ENABLE_PN            2
#define M3_ENABLE_PORT          port(M3_ENABLE_PN)
#define M3_ENABLE_PIN           12
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PN          0
#define M4_STEP_PORT        port(M4_STEP_PN)
#define M4_STEP_PIN         1
#define M4_DIRECTION_PN     0
#define M4_DIRECTION_PORT   port(M4_DIRECTION_PN)
#define M4_DIRECTION_PIN    0
#define M4_ENABLE_PN        0
#define M4_ENABLE_PORT      port(M4_ENABLE_PN)
#define M4_ENABLE_PIN       10
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PN       1
#define SPINDLE_ENABLE_PORT     port(SPINDLE_ENABLE_PN)
#define SPINDLE_ENABLE_PIN      23
#define SPINDLE_DIRECTION_PN    1
#define SPINDLE_DIRECTION_PORT  port(SPINDLE_DIRECTION_PN)
#define SPINDLE_DIRECTION_PIN   21

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
#define SPINDLE_DIRECTION_PIN           21
#else
#define AUXOUTPUT1_PN                   1
#define AUXOUTPUT1_PORT                 port(AUXOUTPUT1_PN)
#define AUXOUTPUT1_PIN                  21
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PN               1
#define SPINDLE_ENABLE_PORT             port(SPINDLE_ENABLE_PN)
#define SPINDLE_ENABLE_PIN              23
#else
#define AUXOUTPUT2_PN                   1
#define AUXOUTPUT2_PORT                 port(AUXOUTPUT2_PN)
#define AUXOUTPUT2_PIN                  23
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PN        3
#define COOLANT_FLOOD_PORT      port(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_PIN       25

#define COOLANT_MIST_PN         3
#define COOLANT_MIST_PORT       port(COOLANT_MIST_PN)
#define COOLANT_MIST_PIN        26

// Define user-control CONTROLs (reset, feed hold, cycle start) input pins.
#define RESET_PORT_PN           0
#define RESET_PORT              port(RESET_PORT_PN)
#define RESET_PIN               18

#define FEED_HOLD_PN            0
#define FEED_HOLD_PORT          port(FEED_HOLD_PN)
#define FEED_HOLD_PIN           16

#define CYCLE_START_PN          0
#define CYCLE_START_PORT        port(CYCLE_START_PN)
#define CYCLE_START_PIN         15

#define CONTROL_INMODE          GPIO_BITBAND

#define AUXINPUT0_PN            2
#define AUXINPUT0_PORT          port(AUXINPUT0_PN)
#define AUXINPUT0_PIN           0
#define AUXINPUT1_PN            0
#define AUXINPUT1_PORT          port(AUXINPUT1_PN)
#define AUXINPUT1_PIN           17

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if MPG_ENABLE == 1
#define MPG_MODE_PORT           AUXINPUT0_PORT
#define MPG_MODE_PIN            AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
#define SD_SPI_PORT 1
#define SD_CS_PN    0
#define SD_CS_PORT  port(SD_CS_PN)
#define SD_CS_PIN   6
#endif

#if TRINAMIC_UART_ENABLE

#define MOTOR_UARTX_PN          1
#define MOTOR_UARTX_PORT        port(MOTOR_UARTX_PN)
#define MOTOR_UARTX_PIN         17
#define MOTOR_UARTY_PN          1
#define MOTOR_UARTY_PORT        port(MOTOR_UARTY_PN)
#define MOTOR_UARTY_PIN         15
#define MOTOR_UARTZ_PN          1
#define MOTOR_UARTZ_PORT        port(MOTOR_UARTZ_PN)
#define MOTOR_UARTZ_PIN         10
#ifdef M3_AVAILABLE
#define MOTOR_UARTM3_PN         1
#define MOTOR_UARTM3_PORT       port(MOTOR_UARTM3_PN)
#define MOTOR_UARTM3_PIN        8
#endif
#ifdef M4_AVAILABLE
#define MOTOR_UARTM4_PN         1
#define MOTOR_UARTM4_PORT       port(MOTOR_UARTM4_PN)
#define MOTOR_UARTM4_PIN        1
#endif

#elif TRINAMIC_SPI_ENABLE

#define TRINAMIC_MOSI_PN        4
#define TRINAMIC_MOSI_PORT      port(TRINAMIC_MOSI_PN)
#define TRINAMIC_MOSI_PIN       28
#define TRINAMIC_MISO_PN        0
#define TRINAMIC_MISO_PORT      port(TRINAMIC_MISO_PN)
#define TRINAMIC_MISO_PIN       5
#define TRINAMIC_SCK_PN         0
#define TRINAMIC_SCK_PORT       port(TRINAMIC_SCK_PN)
#define TRINAMIC_SCK_PIN        4

#define MOTOR_CS_PN 1
#define MOTOR_CS_PORT           port(MOTOR_CS_PN)
#define MOTOR_CSX_PIN           17
#define MOTOR_CSY_PIN           15
#define MOTOR_CSZ_PIN           10
#ifdef M3_AVAILABLE
#define MOTOR_CSM3_PIN          8
#endif
#ifdef M4_AVAILABLE
#define MOTOR_CSM4_PIN          1
#endif

#endif

/**/
