/*

  driver.c - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver.h"
#include "serial.h"
#include "grbl-lpc/pwm_driver.h"

#include "grbl/limits.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if I2C_ENABLE
#include "i2c.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

static bool pwmEnabled = false, IOInitDone = false;
static uint16_t pulse_length, pulse_delay;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static probe_state_t probe = {
    .connected = On
};

static axes_signals_t next_step_outbits;
static pin_group_pins_t limit_inputs = {0};
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

#define DEBOUNCE_QUEUE 8 // Must be a power of 2

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    input_signal_t *signal[DEBOUNCE_QUEUE];
} debounce_queue_t;

#ifdef LIMITS_POLL_PORT
static bool limits_debounce = false;
#endif
static uint32_t limits_invert;
static volatile uint32_t elapsed_tics = 0;
static debounce_queue_t debounce_queue = {0};
static input_signal_t gpio0_signals[10] = {0}, gpio1_signals[10] = {0}, gpio2_signals[10] = {0};

static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
  #ifdef PROBE_PIN
    { .id = Input_Probe,        .port = PROBE_PORT,        .pin = PROBE_PIN,         .group = PinGroup_Probe },
  #endif
  #ifdef RESET_PIN
    { .id = Input_Reset,        .port = RESET_PORT,        .pin = RESET_PIN,         .group = PinGroup_Control },
  #endif
  #ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,     .port = FEED_HOLD_PORT,    .pin = FEED_HOLD_PIN,     .group = PinGroup_Control },
  #endif
  #ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,   .port = CYCLE_START_PORT,  .pin = CYCLE_START_PIN,   .group = PinGroup_Control },
  #endif
  #ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,   .port = SAFETY_DOOR_PORT,  .pin = SAFETY_DOOR_PIN,   .group = PinGroup_Control },
  #endif
    { .id = Input_LimitX,       .port = X_LIMIT_PORT,      .pin = X_LIMIT_PIN,       .group = PinGroup_Limit },
  #ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,     .port = X2_LIMIT_PORT,     .pin = X2_LIMIT_PIN,      .group = PinGroup_Limit },
  #endif
  #ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,   .port = X_LIMIT_PORT_MAX,  .pin = X_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
  #endif
    { .id = Input_LimitY,       .port = Y_LIMIT_PORT,      .pin = Y_LIMIT_PIN,       .group = PinGroup_Limit },
  #ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,     .port = Y2_LIMIT_PORT,     .pin = Y2_LIMIT_PIN,      .group = PinGroup_Limit },
  #endif
  #ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,   .port = Y_LIMIT_PORT_MAX,  .pin = Y_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
  #endif
    { .id = Input_LimitZ,       .port = Z_LIMIT_PORT,      .pin = Z_LIMIT_PIN,       .group = PinGroup_Limit },
  #ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,     .port = Z2_LIMIT_PORT,     .pin = Z2_LIMIT_PIN,      .group = PinGroup_Limit },
  #endif
  #ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,   .port = Z_LIMIT_PORT_MAX,  .pin = Z_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
  #endif
  #ifdef A_LIMIT_PIN
    { .id = Input_LimitA,       .port = A_LIMIT_PORT,      .pin = A_LIMIT_PIN,       .group = PinGroup_Limit },
  #endif
  #ifdef A_LIMIT_PIN_MAX
    { .id = Input_LimitA_Max,   .port = A_LIMIT_PORT_MAX,  .pin = A_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
  #endif
  #ifdef B_LIMIT_PIN
    { .id = Input_LimitB,       .port = B_LIMIT_PORT,      .pin = B_LIMIT_PIN,       .group = PinGroup_Limit },
  #endif
  #ifdef B_LIMIT_PIN_MAX
    { .id = Input_LimitB_Max,   .port = B_LIMIT_PORT_MAX,  .pin = B_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
  #endif
  #ifdef C_LIMIT_PIN
    { .id = Input_LimitC,       .port = C_LIMIT_PORT,      .pin = C_LIMIT_PIN,       .group = PinGroup_Limit },
  #endif
  #ifdef C_LIMIT_PIN_MAX
    { .id = Input_LimitC_Max,   .port = C_LIMIT_PORT_MAX,  .pin = C_LIMIT_PIN_MAX,   .group = PinGroup_Limit },
  #endif
  #if KEYPAD_ENABLE
    { .id = Input_KeypadStrobe, .port = KEYPAD_STROBE_PORT .pin = KEYPAD_STROBE_PIN, .group = PinGroup_Keypad },
  #endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,         .port = AUXINPUT0_PORT,    .pin = AUXINPUT0_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,         .port = AUXINPUT1_PORT,    .pin = AUXINPUT1_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,         .port = AUXINPUT2_PORT,    .pin = AUXINPUT2_PIN,     .group = PinGroup_AuxInput }
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,            .pin = X_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepY,           .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepZ,           .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,            .pin = A_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,            .pin = B_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = C_STEP_PORT,            .pin = C_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = X2_STEP_PORT,           .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = Y2_STEP_PORT,           .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = Z2_STEP_PORT,           .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = X2_DIRECTION_PORT,      .pin = X2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = Y2_DIRECTION_PORT,      .pin = Y2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = Z2_DIRECTION_PORT,      .pin = Z2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable },
#else
#ifdef X_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X_ENABLE_PORT,          .pin = X_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef Y_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y_ENABLE_PORT,          .pin = Y_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef X2_ENABLE_PIN
    { .id = Output_StepperEnableX,  .port = X2_ENABLE_PORT,         .pin = X2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Y2_ENABLE_PIN
    { .id = Output_StepperEnableY,  .port = Y2_ENABLE_PORT,         .pin = Y2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Z2_ENABLE_PIN
    { .id = Output_StepperEnableZ,  .port = Z2_ENABLE_PORT,         .pin = Z2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#endif
#ifdef MOTOR_CS_PIN
    { .id = Output_MotorChipSelect,     .port = MOTOR_CS_PORT,      .pin = MOTOR_CS_PIN,            .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSX_PIN
    { .id = Output_MotorChipSelectX,    .port = MOTOR_CSX_PORT,     .pin = MOTOR_CSX_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSY_PIN
    { .id = Output_MotorChipSelectY,    .port = MOTOR_CSY_PORT,     .pin = MOTOR_CSY_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSZ_PIN
    { .id = Output_MotorChipSelectZ,    .port = MOTOR_CSZ_PORT,     .pin = MOTOR_CSZ_PIN,           .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM3_PIN
    { .id = Output_MotorChipSelectM3,   .port = MOTOR_CSM3_PORT,    .pin = MOTOR_CSM3_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM4_PIN
    { .id = Output_MotorChipSelectM4,   .port = MOTOR_CSM4_PORT,    .pin = MOTOR_CSM4_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#ifdef MOTOR_CSM5_PIN
    { .id = Output_MotorChipSelectM5,   .port = MOTOR_CSM5_PORT,    .pin = MOTOR_CSM5_PIN,          .group = PinGroup_MotorChipSelect },
#endif
#if !VFD_SPINDLE
#ifdef SPINDLE_ENABLE_PIN
    { .id = Output_SpindleOn,       .port = SPINDLE_ENABLE_PORT,    .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
#endif
#ifdef SPINDLE_DIRECTION_PIN
    { .id = Output_SpindleDir,      .port = SPINDLE_DIRECTION_PORT, .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
#endif
#endif
    { .id = Output_CoolantFlood,    .port = COOLANT_FLOOD_PORT,     .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
#ifdef COOLANT_MIST_PIN
    { .id = Output_CoolantMist,     .port = COOLANT_MIST_PORT,      .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#endif
#ifdef SD_CS_PORT
    { .id = Output_SdCardCS,        .port = SD_CS_PORT,             .pin = SD_CS_PIN,               .group = PinGroup_SdCard },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput }
#endif
};

#include "grbl/stepdir_map.h"

#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

static void spindle_set_speed (uint_fast16_t pwm_value);

// Interrupt handler prototypes

uint32_t cpt;

static void driver_delay (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#ifdef STEPPERS_ENABLE_PORT
    DIGITAL_OUT(STEPPERS_ENABLE_PORT, STEPPERS_ENABLE_BIT, enable.x);
#else
    DIGITAL_OUT(X_ENABLE_PORT, X_ENABLE_BIT, enable.x);
  #ifdef X2_ENABLE_PIN
    DIGITAL_OUT(X2_ENABLE_PORT, X2_ENABLE_BIT, enable.x);
  #endif
    DIGITAL_OUT(Y_ENABLE_PORT, Y_ENABLE_BIT, enable.y);
  #ifdef Y2_ENABLE_PIN
    DIGITAL_OUT(Y2_ENABLE_PORT, Y2_ENABLE_BIT, enable.y);
  #endif
    DIGITAL_OUT(Z_ENABLE_PORT, Z_ENABLE_BIT, enable.z);
  #ifdef Z2_ENABLE_PIN
    DIGITAL_OUT(Z2_ENABLE_PORT, Z2_ENABLE_BIT, enable.z);
  #endif
  #ifdef A_AXIS
    DIGITAL_OUT(A_ENABLE_PORT, A_ENABLE_BIT, enable.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_ENABLE_PORT, B_ENABLE_BIT, enable.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_ENABLE_PORT, C_ENABLE_BIT, enable.c);
  #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->TCR = 0b10;          // reset
    STEPPER_TIMER->MR[0] = 0xFFFF;      // Set a long initial delay,
    STEPPER_TIMER->TCR = 0b01;          // start stepper ISR timer in up mode
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals) {
    STEPPER_TIMER->TCR = 0;   // Stop stepper timer
}

// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER->MR[0] = cycles_per_tick < (1UL << 20) ? cycles_per_tick : 0x000FFFFFUL;
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...

#ifdef SQUARING_ENABLED

inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;
    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;

#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_outbits_1.x);
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_outbits_1.y);
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_outbits_1.z);
  #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_outbits_1.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_outbits_1.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_BIT, step_outbits_1.c);
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | step_outmap[step_outbits_1.value & motors_1.mask];
#else
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | (((step_outbits_1.value & motors_1.mask) << STEP_OUTMODE) ^ settings.steppers.step_invert.value);
#endif

#ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_outbits_2.x);
#endif
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_outbits_2.y);
#endif
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_outbits_2.z);
#endif
}

static axes_signals_t getAutoSquaredAxes (void)
{
    axes_signals_t ganged = {0};

    #if X_AUTO_SQUARE
        ganged.x = On;
    #endif

    #if Y_AUTO_SQUARE
        ganged.y = On;
    #endif

    #if Z_AUTO_SQUARE
        ganged.z = On;
    #endif

    return ganged;
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else

inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.value;
    DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_outbits.x);
  #ifdef X2_STEP_PIN
    DIGITAL_OUT(X2_STEP_PORT, X2_STEP_BIT, step_outbits.x);
  #endif
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_outbits.y);
  #ifdef Y2_STEP_PIN
    DIGITAL_OUT(Y2_STEP_PORT, Y2_STEP_BIT, step_outbits.y);
  #endif
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_outbits.z);
  #ifdef Z2_STEP_PIN
    DIGITAL_OUT(Z2_STEP_PORT, Z2_STEP_BIT, step_outbits.z);
  #endif
  #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_outbits.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_outbits.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_BIT, step_outbits_1.c);
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | step_outmap[step_outbits.value];
 #if N_GANGED
  #ifdef X2_STEP_PIN
    BITBAND_PERI(X2_STEP_PORT->ODR, X2_STEP_BIT) = step_outbits.x ^ settings.steppers.step_invert.x;
  #endif
  #ifdef Y2_STEP_PIN
    BITBAND_PERI(Y2_STEP_PORT->ODR, Y2_STEP_BIT) = step_outbits.y ^ settings.steppers.step_invert.y;
  #endif
  #ifdef Z2_STEP_PIN
    BITBAND_PERI(Z2_STEP_PORT->ODR, Z2_STEP_BIT) = step_outbits.z ^ settings.steppers.step_invert.z;
  #endif
 #endif
#else
 #if N_GANGED
    step_outbits.mask ^= settings.steppers.step_invert.mask;
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | (step_outbits.mask << STEP_OUTMODE);
  #ifdef X2_STEP_PIN
    BITBAND_PERI(X2_STEP_PORT->ODR, X2_STEP_BIT) = step_outbits.x;
  #endif
  #ifdef Y2_STEP_PIN
    BITBAND_PERI(Y2_STEP_PORT->ODR, Y2_STEP_BIT) = step_outbits.y;
  #endif
  #ifdef Z2_STEP_PIN
    BITBAND_PERI(Z2_STEP_PORT->ODR, Z2_STEP_BIT) = step_outbits.z;
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE));
  #endif
 #else
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.value);
 #endif
#endif
}

#endif

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.value;
    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_BIT, dir_outbits.x);
    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_BIT, dir_outbits.y);
    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_BIT, dir_outbits.z);
  #ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_BIT, dir_outbits.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_BIT, dir_outbits.b);
  #endif
  #ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_BIT, dir_outbits.c);
  #endif
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->PIN = (DIRECTION_PORT->PIN & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->PIN = (DIRECTION_PORT->PIN & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.steppers.dir_invert.value);
#endif
#ifdef X2_DIRECTION_PIN
  DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_BIT, dir_outbits.x);
#endif
#ifdef Y2_DIRECTION_PIN
  DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_BIT, dir_outbits.y);
#endif
#ifdef Z2_DIRECTION_PIN
  DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_BIT, dir_outbits.z);
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        stepperSetDirOutputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->TCR = 0b01;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_change) {

        stepperSetDirOutputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {
            PULSE_TIMER->TCR = 0b10;
            next_step_outbits = stepper->step_outbits; // Store out_bits
            PULSE_TIMER->MR[0] = pulse_delay;
            PULSE_TIMER->TCR = 0b01;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->TCR = 1;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
#ifndef LIMITS_POLL_PORT
    uint32_t i = limit_inputs.n_pins;

    on &= settings.limits.flags.hard_enabled;

    do {
        i--;
        if(on)
            gpio_int_enable(&limit_inputs.pins.inputs[i], limit_inputs.pins.inputs[i].irq_mode);  // Enable interrupt.
        else
            gpio_int_enable(&limit_inputs.pins.inputs[i], IRQ_Mode_None); // Disable interrupt.
    } while(i);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif

#if LIMIT_INMODE == LIMIT_SHIFT
    signals.value = (uint32_t)(LIMIT_PORT->PIN & LIMIT_MASK) >> LIMIT_SHIFT;
#elif LIMIT_INMODE == GPIO_BITBAND
    signals.min.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_BIT);
    signals.min.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_BIT);
    signals.min.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_BIT);
  #ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(A_LIMIT_PORT, A_LIMIT_BIT);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(B_LIMIT_PORT, B_LIMIT_BIT);
  #endif
#else
    uint32_t bits = LIMIT_PORT->PIN;
    signals.min.x = (bits & X_LIMIT_BIT) != 0;
    signals.min.y = (bits & Y_LIMIT_BIT) != 0;
    signals.min.z = (bits & Z_LIMIT_BIT) != 0;
  #ifdef A_LIMIT_PIN
    signals.min.a = (bits & A_LIMIT_BIT) != 0;
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = (bits & B_LIMIT_BIT) != 0;
  #endif
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(X2_LIMIT_PORT, X2_LIMIT_BIT);
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(Y2_LIMIT_PORT, Y2_LIMIT_BIT);
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(Z2_LIMIT_PORT, Z2_LIMIT_BIT);
#endif

#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PORT_MAX, X_LIMIT_BIT_MAX);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PORT_MAX, Y_LIMIT_BIT_MAX);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PORT_MAX, Z_LIMIT_BIT_MAX);
#endif

    if (settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.mask;

#if CONTROL_INMODE == GPIO_BITBAND
    signals.reset = DIGITAL_IN(RESET_PORT, RESET_BIT);
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, FEED_HOLD_BIT);
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, CYCLE_START_BIT);
  #ifdef SAFETY_DOOR_PORT
    signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_BIT);
  #endif
#else
    uint8_t bits = CONTROL_PORT->PIN;
    signals.reset = bits & RESET_BIT != 0;
    signals.safety_door_ajar = bits & SAFETY_DOOR_BIT != 0;
    signals.feed_hold = bits & FEED_HOLD_BIT != 0;
    signals.cycle_start = bits & CYCLE_START_BIT != 0;
#endif
    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

#ifndef SAFETY_DOOR_PORT
    signals.safety_door_ajar = 0;
#endif

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = settings.probe.invert_probe_pin;

    if (is_probe_away)
        probe.inverted = !probe.inverted;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = !!(PROBE_PORT->PIN & PROBE_BIT) ^ probe.inverted;

    return state;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, settings.spindle.invert.on);
}

inline static void spindle_on (void)
{
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, !settings.spindle.invert.on);
}

inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir)
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, ccw ^ settings.spindle.invert.ccw);
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

// Variable spindle control functions

// Sets spindle speed
static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
        if(spindle_pwm.always_on) {
            pwm_set_width(&SPINDLE_PWM_CHANNEL, spindle_pwm.off_value);
            pwm_enable(&SPINDLE_PWM_CHANNEL);
        } else {
            pwm_set_width(&SPINDLE_PWM_CHANNEL, 0);
//          pwm_disable(&SPINDLE_PWM_CHANNEL); // Set PWM output low
        }
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        pwm_set_width(&SPINDLE_PWM_CHANNEL, pwm_value);
        pwm_enable(&SPINDLE_PWM_CHANNEL);
    }
}

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else

static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

#endif

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    if (!state.on || rpm == 0.0f) {
        spindle_set_speed(spindle_pwm.off_value);
        spindle_off();
    } else {
        if(hal.driver_cap.spindle_dir)
            spindle_dir(state.ccw);

        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
    }
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {settings.spindle.invert.mask};

    state.on = (SPINDLE_ENABLE_PORT->PIN & SPINDLE_ENABLE_BIT) != 0;
    state.ccw = hal.driver_cap.spindle_dir && (SPINDLE_DIRECTION_PORT->PIN & SPINDLE_DIRECTION_BIT) != 0;
    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.value = On;

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;

    DIGITAL_OUT(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, mode.flood);

#ifdef COOLANT_MIST_PORT
    DIGITAL_OUT(COOLANT_MIST_PORT, COOLANT_MIST_BIT, mode.mist);
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {settings.coolant_invert.mask};

    state.flood = DIGITAL_IN(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT);
#ifdef COOLANT_MIST_PORT
    state.mist  = DIGITAL_IN(COOLANT_MIST_PORT, COOLANT_MIST_BIT);;
#endif
    state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();

    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();

    return prev;
}

inline static uint8_t gpio_to_pn (LPC_GPIO_T *port)
{
    return ((uint32_t)port - LPC_GPIO0_BASE) / sizeof(LPC_GPIO_T);
}

void gpio_int_enable (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    if(input->port == LPC_GPIO0) {

        LPC_GPIOINT->IO0.CLR = input->bit;

        switch(irq_mode) {
            case IRQ_Mode_Falling:
                LPC_GPIOINT->IO0.ENR &= ~input->bit;
                LPC_GPIOINT->IO0.ENF |= input->bit;
                break;
            case IRQ_Mode_Rising:
                LPC_GPIOINT->IO0.ENR |= input->bit;
                LPC_GPIOINT->IO0.ENF &= ~input->bit;
                break;
            case IRQ_Mode_Change:
                LPC_GPIOINT->IO0.ENR |= input->bit;
                LPC_GPIOINT->IO0.ENF |= input->bit;
                break;
            default:
                LPC_GPIOINT->IO0.ENR &= ~input->bit;
                LPC_GPIOINT->IO0.ENF &= ~input->bit;
                break;
        }

    } else {

        LPC_GPIOINT->IO2.CLR = input->bit;

        switch(irq_mode) {
            case IRQ_Mode_Falling:
                LPC_GPIOINT->IO2.ENR &= ~input->bit;
                LPC_GPIOINT->IO2.ENF |= input->bit;
                break;
            case IRQ_Mode_Rising:
                LPC_GPIOINT->IO2.ENR |= input->bit;
                LPC_GPIOINT->IO2.ENF &= ~input->bit;
                break;
            case IRQ_Mode_Change:
                LPC_GPIOINT->IO2.ENR |= input->bit;
                LPC_GPIOINT->IO2.ENF |= input->bit;
                break;
            default:
                LPC_GPIOINT->IO2.ENR &= ~input->bit;
                LPC_GPIOINT->IO2.ENF &= ~input->bit;
                break;
        }
    }
}

uint32_t getElapsedTicks (void)
{
    return elapsed_tics;
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    hal.driver_cap.variable_spindle = spindle_precompute_pwm_values(&spindle_pwm, SystemCoreClock / Chip_Clock_GetPCLKDiv(SYSCTL_PCLK_PWM1));

#if USE_STEPDIR_MAP
    stepdirmap_init (settings);
#endif

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

        if(hal.driver_cap.variable_spindle) {
            pwm_init(&SPINDLE_PWM_CHANNEL, SPINDLE_PWM_USE_PRIMARY_PIN, SPINDLE_PWM_USE_SECONDARY_PIN, spindle_pwm.period, 0);
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;

        int32_t t = (uint32_t)(12.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;
        pulse_length = t < 2 ? 2 : t;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            t = (uint32_t)(12.0f * (settings->steppers.pulse_delay_microseconds - 1.5f)) - 1;
            pulse_delay = t < 2 ? 2 : t;
            if(pulse_delay == pulse_length)
                pulse_delay++;
            hal.stepper.pulse_start = &stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = &stepperPulseStart;

        PULSE_TIMER->TCR = 0b10;
        PULSE_TIMER->MR[0] = pulse_length;
        PULSE_TIMER->MCR |= (MR0I|MR0S|MR0R); // Enable interrupt for finish step pulse, reset and stop timer
        PULSE_TIMER->TCR = 0b00;

        stepperSetStepOutputs((axes_signals_t){0});

        NVIC_DisableIRQ(EINT3_IRQn);  // Disable GPIO interrupt

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup = true;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        input_signal_t *input;
        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t), a = 0, b = 0, c = 0;

        limits_invert = 0;

        do {

            LPC_GPIOINT->IO0.CLR = 0xFFFF;
            LPC_GPIOINT->IO2.CLR = 0xFFFF;

            input = &inputpin[--i];

            if(input->port != NULL) {

                input->bit = 1U << input->pin;

                switch(input->id) {

                    case Input_Reset:
                        pullup = !settings->control_disable_pullup.reset;
                        input->irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_FeedHold:
                        pullup = !settings->control_disable_pullup.feed_hold;
                        input->irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_CycleStart:
                        pullup = !settings->control_disable_pullup.cycle_start;
                        input->irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_SafetyDoor:
                        pullup = !settings->control_disable_pullup.safety_door_ajar;
                        input->irq_mode = control_fei.safety_door_ajar ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_Probe:
                        pullup = hal.driver_cap.probe_pull_up;
                        input->irq_mode = IRQ_Mode_None;
                        break;

                    case Input_LimitX:
                    case Input_LimitX_2:
                    case Input_LimitX_Max:
                        if(settings->limits.invert.x)
                            limits_invert |= input->bit;
                        pullup = !settings->limits.disable_pullup.x;
                        input->irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitY:
                    case Input_LimitY_2:
                    case Input_LimitY_Max:
                        if(settings->limits.invert.y)
                            limits_invert |= input->bit;
                        pullup = !settings->limits.disable_pullup.y;
                        input->irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitZ:
                    case Input_LimitZ_2:
                    case Input_LimitZ_Max:
                        if(settings->limits.invert.z)
                            limits_invert |= input->bit;
                        pullup = !settings->limits.disable_pullup.z;
                        input->irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitA:
                    case Input_LimitA_Max:
                        if(settings->limits.invert.a)
                            limits_invert |= input->bit;
                        pullup = !settings->limits.disable_pullup.a;
                        input->irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitB:
                    case Input_LimitB_Max:
                        if(settings->limits.invert.b)
                            limits_invert |= input->bit;
                        pullup = !settings->limits.disable_pullup.b;
                        input->irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitC:
                    case Input_LimitC_Max:
                        if(settings->limits.invert.c)
                            limits_invert |= input->bit;
                        pullup = !settings->limits.disable_pullup.c;
                        input->irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_KeypadStrobe:
                        pullup = true;
                        input->irq_mode = IRQ_Mode_Change; // -> any edge?
                        break;

                    default:
                        pullup = false;
                        input->irq_mode = IRQ_Mode_None;
                        break;
                }

                if(input->group == PinGroup_AuxInput) {
                    pullup = true;
                    input->cap.pull_mode = (PullMode_Up|PullMode_Down);
                    input->cap.irq_mode = input->port == LPC_GPIO0 || input->port == LPC_GPIO2 ? (IRQ_Mode_Rising|IRQ_Mode_Falling|IRQ_Mode_Change) : IRQ_Mode_None;
                }

                input->port->DIR &= ~input->bit;
                Chip_IOCON_PinMuxSet((LPC_IOCON_T *)LPC_IOCON_BASE, gpio_to_pn(input->port), input->pin, pullup ? IOCON_MODE_PULLUP : IOCON_MODE_PULLDOWN);

                // GPIO1, GPIO3 and GPIO4 are not interrupt capable ports
                if(input->port == LPC_GPIO3 || input->port == LPC_GPIO4) {

                    if(input->irq_mode != IRQ_Mode_None) {
                        hal.stream.write("[MSG:Bad bin configuration]" ASCII_EOL);
                        while(true);
                    }

                    if(input->group == PinGroup_Limit)
                        input->irq_mode = IRQ_Mode_None;
                }

                if(input->irq_mode != IRQ_Mode_None || input->group == PinGroup_AuxInput) {

                    input->debounce = hal.driver_cap.software_debounce && !(input->group == PinGroup_Probe || input->group == PinGroup_Keypad || input->group == PinGroup_AuxInput);

                    if(input->port == LPC_GPIO0) {
                        gpio_int_enable(input, input->irq_mode);
                        memcpy(&gpio0_signals[a++], &inputpin[i], sizeof(input_signal_t));
                    } else if(input->port == LPC_GPIO1) { // Limit pins are polled
                        memcpy(&gpio1_signals[b++], &inputpin[i], sizeof(input_signal_t));
                    } else if(input->port == LPC_GPIO2) {
                        gpio_int_enable(input, input->irq_mode);
                        memcpy(&gpio2_signals[c++], &inputpin[i], sizeof(input_signal_t));
                    }
                }
            }
        } while(i);

        NVIC_EnableIRQ(EINT3_IRQn);  // Enable GPIO interrupts
    }
}

static inline char *port2char (LPC_GPIO_T *port)
{
    static char s[4]= "P?.";

    s[1] = '0' + gpio_to_pn(port);

    return s;
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info)
{
    static xbar_t pin = {0};
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.pin = inputpin[i].pin;
        pin.bit = inputpin[i].bit;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.description = inputpin[i].description;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;

        pin_info(&pin);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.pin = outputpin[i].pin;
        pin.bit = outputpin[i].bit;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char(ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin);

        ppin = ppin->next;
    } while(ppin);
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        outputpin[i].bit = 1U << outputpin[i].pin;
        // Cleanup after (potentially) sloppy bootloader
        Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, gpio_to_pn(outputpin[i].port), outputpin[i].pin, IOCON_MODE_INACT, IOCON_FUNC0);
        //
        outputpin[i].port->DIR |= outputpin[i].bit;

        if(outputpin[i].group == PinGroup_MotorChipSelect)
            outputpin[i].port->SET = outputpin[i].bit;
    }

    // Stepper init

    Chip_TIMER_Init(STEPPER_TIMER);
    Chip_TIMER_Init(PULSE_TIMER);

    STEPPER_TIMER->TCR = 0;            // disable
    STEPPER_TIMER->CTCR = 0;           // timer mode
    STEPPER_TIMER->PR = 0;             // no prescale
    STEPPER_TIMER->MCR = MR0I|MR0R;    // MR0: !stop, reset, interrupt
    STEPPER_TIMER->CCR = 0;            // no capture
    STEPPER_TIMER->EMR = 0;            // no external match

    uint32_t xx = SystemCoreClock / 12000000UL / Chip_Clock_GetPCLKDiv(PULSE_TIMER_PCLK) - 1; // to 0.12 us

    PULSE_TIMER->TCR = 0b10;
    PULSE_TIMER->CTCR = 0;
    PULSE_TIMER->PR = xx; // to 0.1 us;
    PULSE_TIMER->TCR = 0;

    NVIC_EnableIRQ(STEPPER_TIMER_INT0);   // Enable stepper interrupt
    NVIC_EnableIRQ(PULSE_TIMER_INT0);     // Enable step pulse interrupt

    NVIC_SetPriority(PULSE_TIMER_INT0, 0);
    NVIC_SetPriority(STEPPER_TIMER_INT0, 2);

 // Limit pins init

    NVIC_EnableIRQ(EINT3_IRQn);  // Enable GPIO interrupt

 // Control pins init
 // NOTE: CS is shared with limit isr

//    NVIC_EnableIRQ(CONTROL_INT);  // Enable limit port interrupt

    if(hal.driver_cap.software_debounce) {
        Chip_TIMER_Init(DEBOUNCE_TIMER);
        DEBOUNCE_TIMER->TCR = 0b10;
        DEBOUNCE_TIMER->CTCR = 0;
        DEBOUNCE_TIMER->PR = SystemCoreClock / 1000000 / Chip_Clock_GetPCLKDiv(DEBOUNCE_TIMER_PCLK); // 1 us
        DEBOUNCE_TIMER->MCR |= (MR0I|MR0S);
        DEBOUNCE_TIMER->MR[0] = 4000; // 40 ms
        DEBOUNCE_TIMER->TCR = 0;
        NVIC_EnableIRQ(DEBOUNCE_TIMER_INT0); // Enable debounce interrupt
    }

 // Set defaults

#if N_AXIS > 3
    IOInitDone = settings->version == 20;
#else
    IOInitDone = settings->version == 19;
#endif

    hal.settings_changed(settings);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

#if SDCARD_ENABLE
    DIGITAL_OUT(SD_CS_PORT, SD_CS_BIT, 1);
    sdcard_init();
#endif

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void) {

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    SystemCoreClockUpdate();

    Chip_SetupXtalClocking(); // Sets 96 MHz clock
    Chip_SYSCTL_SetFLASHAccess(FLASHTIM_120MHZ_CPU);

    SystemCoreClockUpdate();

    Chip_GPIO_Init(LPC_GPIO);
    Chip_IOCON_Init(LPC_IOCON);

    // Enable and set SysTick IRQ to lowest priority
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    hal.info = "LCP1769";
    hal.driver_version = "211113";
    hal.driver_setup = driver_setup;
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.f_step_timer = SystemCoreClock / Chip_Clock_GetPCLKDiv(STEPPER_TIMER_PCLK);
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
    hal.stepper.motor_iterator = motor_iterator,
#ifdef SQUARING_ENABLED
    hal.stepper.get_auto_squared = getAutoSquaredAxes;
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;

    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle.update_rpm = spindleUpdateRPM;
#endif

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

#if USB_SERIAL_CDC
    memcpy(&hal.stream, usbInit(), sizeof(io_stream_t));
#else
    memcpy(&hal.stream, serialInit(), sizeof(io_stream_t));
#endif

#if I2C_ENABLE
    i2c_init();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
#else
    hal.nvs.type = NVS_None;
#endif

#ifdef HAS_KEYPAD
    hal.execute_realtime = process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
#ifdef COOLANT_MIST_PORT
    hal.driver_cap.mist_control = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#if SDCARD_ENABLE
    hal.driver_cap.sd_card = On;
#endif

    uint32_t i;
    input_signal_t *input;

#ifdef HAS_IOPORTS
    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};
#endif

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
#ifdef HAS_IOPORTS
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            aux_inputs.n_pins++;
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.irq_mode = input->port == LPC_GPIO0 || input->port == LPC_GPIO2 ? IRQ_Mode_Edges : IRQ_Mode_None;
        }
#endif
        if(input->group == PinGroup_Limit) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        }
    }

#ifdef HAS_IOPORTS
    output_signal_t *output;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            aux_outputs.n_pins++;
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);

#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#include "grbl/plugins_init.h"

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_IRQHandler (void)
{
    STEPPER_TIMER->IR = STEPPER_TIMER->IR;
    hal.stepper.interrupt_callback();
}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/

// This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
// initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
// will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
// The new timing between direction, step pulse, and step complete events are setup in the
// st_wake_up() routine.

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
void STEPPULSE_IRQHandler (void)
{
    PULSE_TIMER->IR = PULSE_TIMER->IR;

    if(PULSE_TIMER->MR[0] == pulse_length)
        stepperSetStepOutputs((axes_signals_t){0}); // End step pulse.
    else {
        stepperSetStepOutputs(next_step_outbits);   // Begin step pulse.
        PULSE_TIMER->TCR = 0b10;
        PULSE_TIMER->MR[0] = pulse_length;
        PULSE_TIMER->TCR = 0b01;
    }
}

inline static bool enqueue_debounce (input_signal_t *signal)
{
    bool ok;
    uint_fast8_t bptr = (debounce_queue.head + 1) & (DEBOUNCE_QUEUE - 1);

    if((ok = bptr != debounce_queue.tail)) {
        debounce_queue.signal[debounce_queue.head] = signal;
        debounce_queue.head = bptr;
    }

    return ok;
}

// Returns NULL if no debounce checks enqueued
inline static input_signal_t *get_debounce (void)
{
    input_signal_t *signal = NULL;
    uint_fast8_t bptr = debounce_queue.tail;

    if(bptr != debounce_queue.head) {
        signal = debounce_queue.signal[bptr++];
        debounce_queue.tail = bptr & (DEBOUNCE_QUEUE - 1);
    }

    return signal;
}

void DEBOUNCE_IRQHandler (void)
{
    input_signal_t *signal;

    DEBOUNCE_TIMER->IR = MR0IFG;
    DEBOUNCE_TIMER->TCR = 0;

    while((signal = get_debounce())) {

        gpio_int_enable(signal, signal->irq_mode);

        if(DIGITAL_IN(signal->port, signal->bit) == (signal->irq_mode == IRQ_Mode_Falling ? 0 : 1))
          switch(signal->group) {

            case PinGroup_Limit:
                {
                    limit_signals_t state = limitsGetState();
                    if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
                        hal.limits.interrupt_callback(state);
                }
                break;

            case PinGroup_Control:
                hal.control.interrupt_callback(systemGetState());
                break;

            default:
                break;
        }
    }
}

void GPIO_IRQHandler (void)
{
    bool debounce = false;
    uint32_t grp = 0, i = 0;
    uint32_t istat = LPC_GPIOINT->STATUS, iflags;

    if(istat & P0Int) {
        iflags = LPC_GPIOINT->IO0.STATR | LPC_GPIOINT->IO0.STATF;
        LPC_GPIOINT->IO0.CLR = iflags;

        while(gpio0_signals[i].port != NULL) {
            if(iflags & gpio0_signals[i].bit) {
                if(gpio0_signals[i].debounce && enqueue_debounce(&gpio0_signals[i])) {
                    gpio_int_enable(&gpio0_signals[i], IRQ_Mode_None);
                    debounce = true;
                } else switch(gpio0_signals[i].group) {
#ifdef HAS_IOPORTS
                    case PinGroup_AuxInput:
                        ioports_event(&gpio0_signals[i]);
                        break;
#endif
#if KEYPAD_ENABLE
                    case PinGroup_Keypad:
                        keypad_keyclick_handler(BITBAND_PERI(KEYPAD_PORT->PIO_PDSR, KEYPAD_PIN));
                        break;
#endif
                    default:
                        grp |= gpio0_signals[i].group;
                        break;
                }
            }
            i++;
        }
    }

    if(istat & P2Int) {
        iflags = LPC_GPIOINT->IO2.STATR | LPC_GPIOINT->IO2.STATF;
        LPC_GPIOINT->IO2.CLR = iflags;

        while(gpio2_signals[i].port != NULL) {
            if(iflags & gpio2_signals[i].bit) {
                if(gpio2_signals[i].debounce && enqueue_debounce(&gpio2_signals[i])) {
                    gpio_int_enable(&gpio2_signals[i], IRQ_Mode_None);
                    debounce = true;
                } else switch(gpio2_signals[i].group) {
#ifdef HAS_IOPORTS
                    case PinGroup_AuxInput:
                        ioports_event(&gpio2_signals[i]);
                        break;
#endif
#if KEYPAD_ENABLE
                    case PinGroup_Keypad:
                        keypad_keyclick_handler(BITBAND_PERI(KEYPAD_PORT->PIO_PDSR, KEYPAD_PIN));
                        break;
#endif
                    default:
                        grp |= gpio2_signals[i].group;
                        break;
                }
            }
            i++;
        }
    }

    if(debounce) {
        // Reset and start
        DEBOUNCE_TIMER->TCR = 0;
        DEBOUNCE_TIMER->TC = 1;
        DEBOUNCE_TIMER->TCR = 0b10;
        while(DEBOUNCE_TIMER->TC != 0);
        DEBOUNCE_TIMER->TCR = 0b01;
    }

    if(grp & PinGroup_Limit)
        hal.limits.interrupt_callback(limitsGetState());

    if(grp & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());
}

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    elapsed_tics++;

#ifdef LIMITS_POLL_PORT // Poll limit pins when hard limits enabled
    static uint32_t limits_state = 0, limits = 0;
    if(settings.limits.flags.hard_enabled) {
        limits = (LIMITS_POLL_PORT->PIN ^ limits_invert) & LIMIT_MASK;
        if(limits_state && limits == 0 && !limits_debounce)
            limits_state = 0;
        else if(limits_state != limits && limits) {

           uint32_t i = 0;
           while(gpio1_signals[i].port != NULL) {
                if(limits & gpio1_signals[i].bit && gpio1_signals[i].debounce && enqueue_debounce(&gpio1_signals[i]))
                    limits_debounce = true;
                i++;
            }

            if(limits_debounce) {
                // Reset and start
                DEBOUNCE_TIMER->TCR = 0;
                DEBOUNCE_TIMER->TC = 1;
                DEBOUNCE_TIMER->TCR = 0b10;
                while(DEBOUNCE_TIMER->TC != 0);
                DEBOUNCE_TIMER->TCR = 0b01;
            } else
                hal.limits.interrupt_callback(limitsGetState());

            limits_state = limits;
        }
    }
#endif

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
