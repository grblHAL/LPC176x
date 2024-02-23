/*

  driver.h - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2018-2024 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/driver_opts.h"

#include "portmacros.h"

// Configuration
// Set value to 1 to enable, 0 to disable

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       0
#endif
#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE       0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM      0
#endif
#ifndef LIMIT_MAX_ENABLE
#define LIMIT_MAX_ENABLE    0
#endif

// Define GPIO output mode options

#define GPIO_SHIFT0  0
#define GPIO_SHIFT1  1
#define GPIO_SHIFT2  2
#define GPIO_SHIFT3  3
#define GPIO_SHIFT4  4
#define GPIO_SHIFT5  5
#define GPIO_MAP     8
#define GPIO_BITBAND 9

// NOTE:
// P0.27, P0.28 are dedicated I2C pins without pull up/down.
// P0.29, P0.30 must have same direction as used for USB operation.

#ifdef SMOOTHIEBOARD
  #include "smoothieboard_map.h"
#elif defined(BOARD_RAMPS_16)
  #include "ramps_1.6_map.h"
#elif defined(BOARD_BTT_SKR_13)
  #include "btt_skr_1.3_map.h"
#elif defined(BOARD_BTT_SKR_14_TURBO)
  #include "btt_skr_1.4_turbo_map.h"
#elif defined BOARD_BTT_SKR_E3_TURBO
  #include "btt_skr_e3_turbo_map.h"
#elif defined(BOARD_MKS_SBASE_13)
  #include "mks_sbase_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else
  #include "generic_map.h"
#endif

#if SAFETY_DOOR_ENABLE && !defined(SAFETY_DOOR_PIN)
#error Safety door input is not supported by the selected board!
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 2.2f // microseconds
#endif

// End configuration

#if MPG_MODE && !USB_SERIAL_CDC
#error "MPG_MODE can only be enabled with USB_SERIAL_CDC enabled!"
#endif

#if MPG_MODE == 1 && !defined(MPG_MODE_PIN)
#error "MPG_MODE_PIN must be defined!"
#endif

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE 0
#endif

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#endif

#ifndef RESET_PORT
#define RESET_PORT CONTROL_PORT
#define RESET_INTENR CONTROL_INTENR
#define RESET_INTENF CONTROL_INTENF
#define RESET_INTCLR CONTROL_INTCLR

#endif
#ifndef FEED_HOLD_PORT
#define FEED_HOLD_PORT CONTROL_PORT
#define FEED_HOLD_INTENR CONTROL_INTENR
#define FEED_HOLD_INTENF CONTROL_INTENF
#define FEED_HOLD_INTCLR CONTROL_INTCLR
#endif
#ifndef CYCLE_START_PORT
#define CYCLE_START_PORT CONTROL_PORT
#define CYCLE_START_INTENR CONTROL_INTENR
#define CYCLE_START_INTENF CONTROL_INTENF
#define CYCLE_START_INTCLR CONTROL_INTCLR
#endif

#ifdef MOTOR_CS_PORT
#if defined(MOTOR_CSX_PIN) & !defined(MOTOR_CSX_PORT)
#define MOTOR_CSX_PORT MOTOR_CS_PORT
#endif
#if defined(MOTOR_CSY_PIN) & !defined(MOTOR_CSY_PORT)
#define MOTOR_CSY_PORT MOTOR_CS_PORT
#endif
#if defined(MOTOR_CSZ_PIN) & !defined(MOTOR_CSZ_PORT)
#define MOTOR_CSZ_PORT MOTOR_CS_PORT
#endif
#if defined(MOTOR_CSM3_PIN) & !defined(MOTOR_CSM3_PORT)
#define MOTOR_CSM3_PORT MOTOR_CS_PORT
#endif
#if defined(MOTOR_CSM4_PIN) & !defined(MOTOR_CSM4_PORT)
#define MOTOR_CSM4_PORT MOTOR_CS_PORT
#endif
#if defined(MOTOR_CSM5_PIN) & !defined(MOTOR_CSM5_PORT)
#define MOTOR_CSM5_PORT MOTOR_CS_PORT
#endif
#endif

typedef struct {
    LPC_GPIO_T *port;
    uint32_t pin;
    uint32_t bit;
    pin_function_t id;
    pin_group_t group;
    bool debounce;
    pin_cap_t cap;
    pin_mode_t mode;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    LPC_GPIO_T *port;
    uint32_t pin;
    uint32_t bit;
    pin_function_t id;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#ifdef HAS_BOARD_INIT
    void board_init (void);
#endif

#ifdef SD_CS_PIN
#define SD_CS_BIT (1<<SD_CS_PIN)
#endif

// Driver initialization entry point

bool driver_init (void);
void gpio_int_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);

#endif // __DRIVER_H__
