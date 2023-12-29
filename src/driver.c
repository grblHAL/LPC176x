/*

  driver.c - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2018-2023 Terje Io

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

#include "grbl/machine_limits.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/state_machine.h"
#include "grbl/protocol.h"

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

static bool IOInitDone = false;
static uint16_t pulse_length, pulse_delay;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static probe_state_t probe = {
    .connected = On
};
#if DRIVER_SPINDLE_ENABLE
static spindle_id_t spindle_id = -1;
#endif
#if DRIVER_SPINDLE_PWM_ENABLE
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;
#define pwm(s) ((spindle_pwm_t *)s->context)
#endif
static axes_signals_t next_step_outbits;
static pin_group_pins_t limit_inputs = {0};
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
#if MPG_MODE == 1
static input_signal_t *mpg_pin = NULL;
#endif
#if AUX_CONTROLS_ENABLED
static input_signal_t *door_pin = NULL;
#endif

#define DEBOUNCE_QUEUE 8 // Must be a power of 2

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    input_signal_t *signal[DEBOUNCE_QUEUE];
} debounce_queue_t;

#ifdef LIMITS_POLL_PORT
static bool limits_debounce = false, limits_poll = false;
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
  #if ESTOP_ENABLE
    { .id = Input_EStop,        .port = RESET_PORT,        .pin = RESET_PIN,         .group = PinGroup_Control },
  #else
    { .id = Input_Reset,        .port = RESET_PORT,        .pin = RESET_PIN,         .group = PinGroup_Control },
  #endif
#endif
#ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,     .port = FEED_HOLD_PORT,    .pin = FEED_HOLD_PIN,     .group = PinGroup_Control },
#endif
#ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,   .port = CYCLE_START_PORT,  .pin = CYCLE_START_PIN,   .group = PinGroup_Control },
#endif
#if SAFETY_DOOR_BIT
    { .id = Input_SafetyDoor,   .port = SAFETY_DOOR_PORT,  .pin = SAFETY_DOOR_PIN,   .group = PinGroup_Control },
#endif
    { .id = Input_LimitX,       .port = X_LIMIT_PORT,      .pin = X_LIMIT_PIN,       .group = PinGroup_Limit },
#ifdef MPG_MODE_PIN
    { .id = Input_ModeSelect,   .port = MPG_MODE_PORT,     .pin = MPG_MODE_PIN,      .group = PinGroup_MPG },
#endif
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,     .port = X2_LIMIT_PORT,     .pin = X2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,   .port = X_LIMIT_PORT_MAX,  .pin = X_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
#endif
    { .id = Input_LimitY,       .port = Y_LIMIT_PORT,      .pin = Y_LIMIT_PIN,       .group = PinGroup_Limit },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,     .port = Y2_LIMIT_PORT,     .pin = Y2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,   .port = Y_LIMIT_PORT_MAX,  .pin = Y_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
#endif
    { .id = Input_LimitZ,       .port = Z_LIMIT_PORT,      .pin = Z_LIMIT_PIN,       .group = PinGroup_Limit },
#ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,     .port = Z2_LIMIT_PORT,     .pin = Z2_LIMIT_PIN,      .group = PinGroup_Limit },
#endif
#ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,   .port = Z_LIMIT_PORT_MAX,  .pin = Z_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
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
#if I2C_STROBE_ENABLE && defined(I2C_STROBE_PIN)
    { .id = Input_I2CStrobe,    .port = I2C_STROBE_PORT,   .pin = I2C_STROBE_PIN,    .group = PinGroup_Keypad },
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
#ifdef MOTOR_UARTX_PIN
    { .id = Bidirectional_MotorUARTX,   .port = MOTOR_UARTX_PORT,   .pin = MOTOR_UARTX_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTY_PIN
    { .id = Bidirectional_MotorUARTY,   .port = MOTOR_UARTY_PORT,   .pin = MOTOR_UARTY_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTZ_PIN
    { .id = Bidirectional_MotorUARTZ,   .port = MOTOR_UARTZ_PORT,   .pin = MOTOR_UARTZ_PIN,         .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM3_PIN
    { .id = Bidirectional_MotorUARTM3,  .port = MOTOR_UARTM3_PORT,  .pin = MOTOR_UARTM3_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM4_PIN
    { .id = Bidirectional_MotorUARTM4,  .port = MOTOR_UARTM4_PORT,  .pin = MOTOR_UARTM4_PIN,        .group = PinGroup_MotorUART },
#endif
#ifdef MOTOR_UARTM5_PIN
    { .id = Bidirectional_MotorUARTM5,  .port = MOTOR_UARTM5_PORT,  .pin = MOTOR_UARTM5_PIN,        .group = PinGroup_MotorUART },
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

// Interrupt handler prototypes

uint32_t cpt;

#if I2C_STROBE_ENABLE

static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok;

    if((ok = irq == IRQ_I2C_Strobe && i2c_strobe.callback == NULL))
        i2c_strobe.callback = handler;

    return ok;
}

#endif


static void driver_delay (uint32_t ms, delay_callback_ptr callback)
{
    if((delay.ms = ms) > 0) {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delay.callback = callback)) {
            while(delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else if(callback)
        callback();
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

    STEPPER_TIMER->TCR = 0b10;                      // reset
    STEPPER_TIMER->MR[0] = hal.f_step_timer / 500;  // ~2ms delay to allow drivers time to wake up.
    STEPPER_TIMER->TCR = 0b01;                      // start stepper ISR timer in up mode
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

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        #if X_AUTO_SQUARE
            ganged.x = On;
        #endif
        #if Y_AUTO_SQUARE
            ganged.y = On;
        #endif
        #if Z_AUTO_SQUARE
            ganged.z = On;
        #endif
    } else {
        #if X_GANGED
            ganged.x = On;
        #endif

        #if Y_GANGED
            ganged.y = On;
        #endif

        #if Z_GANGED
            ganged.z = On;
        #endif
    }

    return ganged;
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
 #ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_BIT, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_BIT, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_BIT, dir_outbits.z);
  #endif
 #endif
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
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_PIN, (dir_outbits.x ^ settings.steppers.dir_invert.x) ^ settings.steppers.ganged_dir_invert.mask.x;
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_PIN, (dir_outbits.y ^ settings.steppers.dir_invert.y) ^ settings.steppers.ganged_dir_invert.mask.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_PIN, (dir_outbits.z ^ settings.steppers.dir_invert.z) ^ settings.steppers.ganged_dir_invert.mask.z;
  #endif
#else
    DIRECTION_PORT->PIN = (DIRECTION_PORT->PIN & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.steppers.dir_invert.value);
 #ifdef GANGING_ENABLED
   dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
   DIGITAL_OUT(X2_DIRECTION_PORT, X2_DIRECTION_BIT, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
   DIGITAL_OUT(Y2_DIRECTION_PORT, Y2_DIRECTION_BIT, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
   DIGITAL_OUT(Z2_DIRECTION_PORT, Z2_DIRECTION_BIT, dir_outbits.z);
  #endif
 #endif
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
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
#ifdef LIMITS_POLL_PORT
    limits_poll = on && homing_cycle.mask == 0;
#else
    bool disable = !on;
    uint32_t i = limit_inputs.n_pins;
    axes_signals_t pin;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        i--;
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit_inputs.pins.inputs[i].id);
            disable = limit_inputs.pins.inputs[i].group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        if(disable)
            gpio_int_enable(&limit_inputs.pins.inputs[i], IRQ_Mode_None); // Disable interrupt.
        else
            gpio_int_enable(&limit_inputs.pins.inputs[i], limit_inputs.pins.inputs[i].irq_mode);  // Enable interrupt.
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
  #if ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_PORT, RESET_BIT);
  #else
    signals.reset = DIGITAL_IN(RESET_PORT, RESET_BIT);
  #endif
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, FEED_HOLD_BIT);
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, CYCLE_START_BIT);
#else
    uint8_t bits = CONTROL_PORT->PIN;
  #if ESTOP_ENABLE
    signals.e_stop = bits & RESET_BIT != 0;
  #else
    signals.reset = bits & RESET_BIT != 0;
  #endif
    signals.feed_hold = bits & FEED_HOLD_BIT != 0;
    signals.cycle_start = bits & CYCLE_START_BIT != 0;
#endif

#if AUX_CONTROLS_ENABLED

  #ifdef SAFETY_DOOR_PIN
    if(aux_ctrl[AuxCtrl_SafetyDoor].debouncing)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, 1 << SAFETY_DOOR_PIN);
  #endif
  #ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PORT, 1 << MOTOR_FAULT_PIN);
  #endif
  #ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(MOTOR_WARNING_PORT, 1 << MOTOR_WARNING_PIN);
  #endif

  #if AUX_CONTROLS_SCAN
    uint_fast8_t i;
    for(i = AUX_CONTROLS_SCAN; i < AuxCtrl_NumEntries; i++) {
        if(aux_ctrl[i].enabled) {
            signals.mask &= ~aux_ctrl[i].cap.mask;
            if(hal.port.wait_on_input(Port_Digital, aux_ctrl[i].port, WaitMode_Immediate, 0.0f) == 1)
                signals.mask |= aux_ctrl[i].cap.mask;
        }
    }
  #endif

#endif // AUX_CONTROLS_ENABLED

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return signals;
}

#if AUX_CONTROLS_ENABLED

static void aux_irq_handler (uint8_t port, bool state)
{
    uint_fast8_t i;
    control_signals_t signals = systemGetState();

    for(i = 0; i < AuxCtrl_NumEntries; i++) {
        if(aux_ctrl[i].port == port) {
            if(!aux_ctrl[i].debouncing) {
                signals.mask |= aux_ctrl[i].cap.mask;
                if(i == AuxCtrl_SafetyDoor && (aux_ctrl[i].debouncing = enqueue_debounce(door_pin))) {
                    DEBOUNCE_TIMER->TCR = 0;
                    DEBOUNCE_TIMER->TC = 1;
                    DEBOUNCE_TIMER->TCR = 0b10;
                    while(DEBOUNCE_TIMER->TC != 0);
                    DEBOUNCE_TIMER->TCR = 0b01;
                }
            }
        }
    }

    if(signals.mask)
        hal.control.interrupt_callback(signals);
}

bool aux_claim (xbar_t *properties, uint8_t port, void *data)
{
    ((aux_ctrl_t *)data)->port = port;

    return ioport_claim(Port_Digital, Port_Input, &((aux_ctrl_t *)data)->port, xbar_fn_to_pinname(((aux_ctrl_t *)data)->function));
}

static bool aux_claim_explicit (aux_ctrl_t *aux)
{
    if((aux->enabled = aux->port != 0xFF && ioport_claim(Port_Digital, Port_Input, &aux->port, xbar_fn_to_pinname(aux->function))))
        hal.signals_cap.mask |= aux->cap.mask;
    else
        aux->port = 0xFF;

    return aux->enabled;
}

#endif // AUX_CONTROLS_ENABLED

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
static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = !!(PROBE_PORT->PIN & PROBE_BIT) ^ probe.inverted;

    return state;
}

#if DRIVER_SPINDLE_ENABLE

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
#ifdef SPINDLE_DIRECTION_PIN
    DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, ccw ^ settings.spindle.invert.ccw);
#else
    UNUSED(ccw);
#endif
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);

    if (!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

#if DRIVER_SPINDLE_PWM_ENABLE

// Sets spindle speed
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if (pwm_value == pwm(spindle)->off_value) {
        pwmEnabled = false;
        if(pwm(spindle)->settings->flags.enable_rpm_controlled) {
            if(pwm(spindle)->cloned)
                spindle_dir(false);
            else
                spindle_off();
        }
        if(pwm(spindle)->always_on) {
            pwm_set_width(&SPINDLE_PWM_CHANNEL, pwm(spindle)->off_value);
            pwm_enable(&SPINDLE_PWM_CHANNEL);
        } else {
            pwm_set_width(&SPINDLE_PWM_CHANNEL, 0);
//          pwm_disable(&SPINDLE_PWM_CHANNEL); // Set PWM output low
        }
    } else {
        if(!pwmEnabled) {
            if(pwm(spindle)->cloned)
                spindle_dir(true);
            else
                spindle_on();
            pwmEnabled = true;
        }
        pwm_set_width(&SPINDLE_PWM_CHANNEL, pwm_value);
        pwm_enable(&SPINDLE_PWM_CHANNEL);
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return pwm(spindle)->compute_value(pwm(spindle), rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
#ifdef SPINDLE_DIRECTION_PIN
    if (state.on || pwm(spindle)->cloned)
        spindle_dir(state.ccw);
#endif
    if(!pwm(spindle)->settings->flags.enable_rpm_controlled) {
        if(state.on)
            spindle_on();
        else
            spindle_off();
    }

    spindleSetSpeed(spindle, state.on || (state.ccw && pwm(spindle)->cloned)
                              ? pwm(spindle)->compute_value(pwm(spindle), rpm, false)
                              : pwm(spindle)->off_value);
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.spindle, SystemCoreClock / Chip_Clock_GetPCLKDiv(SYSCTL_PCLK_PWM1))) {
        pwm_init(&SPINDLE_PWM_CHANNEL, SPINDLE_PWM_USE_PRIMARY_PIN, SPINDLE_PWM_USE_SECONDARY_PIN, spindle_pwm.period, 0);
        spindle->set_state = spindleSetStateVariable;
    } else {
        if(pwmEnabled)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

#endif // DRIVER_SPINDLE_PWM_ENABLE

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    spindle_state_t state = {settings.spindle.invert.mask};

    state.on = DIGITAL_IN(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT);
#endif
    state.value ^= settings.spindle.invert.mask;

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

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
    state.mist  = DIGITAL_IN(COOLANT_MIST_PORT, COOLANT_MIST_BIT);
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

#if MPG_MODE == 1

static void mpg_select (sys_state_t state)
{
    stream_mpg_enable(DIGITAL_IN(mpg_pin->port, mpg_pin->bit) == 0);

    gpio_int_enable(mpg_pin, mpg_pin->irq_mode = (sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

static void mpg_enable (sys_state_t state)
{
    if(sys.mpg_mode == DIGITAL_IN(mpg_pin->port, mpg_pin->bit))
        stream_mpg_enable(true);

    gpio_int_enable(mpg_pin, mpg_pin->irq_mode = (sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

#endif

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
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
#if USE_STEPDIR_MAP
    stepdirmap_init (settings);
#endif

    if(IOInitDone) {

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

#if DRIVER_SPINDLE_PWM_ENABLE
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif

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
                input->debounce = true;

                switch(input->id) {

                    case Input_Reset:
                        pullup = !settings->control_disable_pullup.reset;
                        input->irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_EStop:
                        pullup = !settings->control_disable_pullup.e_stop;
                        input->irq_mode = control_fei.e_stop ? IRQ_Mode_Falling : IRQ_Mode_Rising;
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
                        input->debounce = false;
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
#if MPG_MODE == 1
                    case Input_ModeSelect:
                        pullup = true;
                        mpg_pin = input;
                        input->irq_mode = IRQ_Mode_Change;
                        input->debounce = false;
                        break;
#endif
                    case Input_I2CStrobe:
                        pullup = true;
                        input->irq_mode = IRQ_Mode_Change; // -> any edge?
                        input->debounce = false;
                        break;

                    default:
                        pullup = false;
                        input->irq_mode = IRQ_Mode_None;
                        break;
                }

                if(input->group == PinGroup_AuxInput) {
                    pullup = true;
                    input->debounce = false;
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

                    if(input->group & (PinGroup_Limit|PinGroup_LimitMax))
                        input->irq_mode = IRQ_Mode_None;
                }

                if(input->irq_mode != IRQ_Mode_None || input->group == PinGroup_AuxInput) {

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

#if AUX_CONTROLS_ENABLED
        for(i = 0; i < AuxCtrl_NumEntries; i++) {
            if(aux_ctrl[i].enabled && aux_ctrl[i].irq_mode != IRQ_Mode_None) {
                aux_ctrl[i].irq_mode = (settings->control_invert.mask & aux_ctrl[i].cap.mask) ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                if(i == AuxCtrl_SafetyDoor)
                    door_pin->irq_mode = aux_ctrl[i].irq_mode;
                hal.port.register_interrupt_handler(aux_ctrl[i].port, aux_ctrl[i].irq_mode, aux_irq_handler);
            }
        }
#endif

        NVIC_EnableIRQ(EINT3_IRQn);  // Enable GPIO interrupts
    }
}

static inline char *port2char (LPC_GPIO_T *port)
{
    static char s[4]= "P?.";

    s[1] = '0' + gpio_to_pn(port);

    return s;
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
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

        pin_info(&pin, data);
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

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char(ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);

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

        if(outputpin[i].group == PinGroup_MotorChipSelect ||
            outputpin[i].group == PinGroup_MotorUART ||
             outputpin[i].group == PinGroup_StepperEnable)
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
    NVIC_SetPriority(STEPPER_TIMER_INT0, 0);

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

    IOInitDone = settings->version == 22;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

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

#if MPG_MODE == 1
    // Drive MPG mode input pin low until setup complete
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, gpio_to_pn(MPG_MODE_PORT), MPG_MODE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    //
    MPG_MODE_PORT->DIR |= (1<<MPG_MODE_PIN);
    DIGITAL_OUT(MPG_MODE_PORT, MPG_MODE_PIN, 0);
#endif

    hal.info = "LCP1769";
    hal.driver_version = "231228";
    hal.driver_setup = driver_setup;
    hal.driver_url = GRBL_URL "/LCP176x";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
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
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

#if USB_SERIAL_CDC
    stream_connect(usbInit());
#else
    stream_connect(serialInit(BAUD_RATE));
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

#if ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
    hal.signals_cap.reset = Off;
#endif
    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
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
    output_signal_t *output;
    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];

        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->id = (pin_function_t)(Input_Aux0 + aux_inputs.n_pins++);
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.irq_mode = input->port == LPC_GPIO0 || input->port == LPC_GPIO2 ? IRQ_Mode_Edges : IRQ_Mode_None;
#if SAFETY_DOOR_ENABLE
            if(input->port == SAFETY_DOOR_PORT && input->pin == SAFETY_DOOR_PIN && input->cap.irq_mode != IRQ_Mode_None) {
                door_pin = input;
                aux_ctrl[AuxCtrl_SafetyDoor].port = aux_inputs.n_pins - 1;
            }
#endif
#if MOTOR_FAULT_ENABLE
            if(input->port == MOTOR_FAULT_PORT && input->pin == MOTOR_FAULT_PIN && input->cap.irq_mode != IRQ_Mode_None)
                aux_ctrl[AuxCtrl_MotorFault].port = aux_inputs.n_pins - 1;
#endif
#if MOTOR_WARNING_ENABLE
            if(input->port == MOTOR_WARNING_PORT && input->pin == MOTOR_WARNING_PIN && input->cap.irq_mode != IRQ_Mode_None)
                aux_control_port[AuxCtrl_MotorWarning] = aux_inputs.n_pins - 1;
#endif
        } else if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        }
    }

    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins++);
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);

#if SAFETY_DOOR_ENABLE
    aux_claim_explicit(&aux_ctrl[AuxCtrl_SafetyDoor]);
#elif defined(SAFETY_DOOR_PIN)
    hal.signals_cap.safety_door = On;
#endif

#if MOTOR_FAULT_ENABLE
    aux_claim_explicit(&aux_ctrl[AuxCtrl_MotorFault]);
#elif defined(MOTOR_FAULT_PIN)
    hal.signals_cap.motor_fault = On;
#endif

#if MOTOR_WARNING_ENABLE
    aux_claim_explicit(&aux_ctrl[AuxCtrl_MotorWarning]);
#elif defined(MOTOR_WARNING_PIN)
    hal.signals_cap.motor_warning = On;
#endif

#if AUX_CONTROLS_ENABLED
    for(i = AuxCtrl_ProbeDisconnect; i < AuxCtrl_NumEntries; i++) {
        if(aux_ctrl[i].enabled) {
            if((aux_ctrl[i].enabled = ioports_enumerate(Port_Digital, Port_Input, (pin_mode_t){ .irq_mode = aux_ctrl[i].irq_mode }, true, aux_claim, (void *)&aux_ctrl[i])))
                hal.signals_cap.mask |= aux_ctrl[i].cap.mask;
        }
    }
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

    serialRegisterStreams();

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_PWM_ENABLE

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

#if MPG_MODE == 1
  #if KEYPAD_ENABLE == 2
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), false, keypad_enqueue_keycode)))
        protocol_enqueue_rt_command(mpg_enable);
  #else
    if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), false, NULL)))
        protocol_enqueue_rt_command(mpg_enable);
  #endif
#elif MPG_MODE == 2
    hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL), false, keypad_enqueue_keycode);
//#elif KEYPAD_ENABLE == 2
//    stream_open_instance(KEYPAD_STREAM, 115200, keypad_enqueue_keycode);
#endif


#include "grbl/plugins_init.h"

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
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
            case PinGroup_LimitMax:
                {
                    limit_signals_t state = limitsGetState();
                    if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
                        hal.limits.interrupt_callback(state);
                }
                break;

            case PinGroup_Control:
                hal.control.interrupt_callback(systemGetState());
                break;

#if AUX_CONTROLS_ENABLED
            case PinGroup_AuxInput:
               aux_ctrl[AuxCtrl_SafetyDoor].debouncing = false;
               break;
#endif

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

#if MPG_MODE == 1
                case PinGroup_MPG:
                    gpio_int_enable(&gpio0_signals[i], IRQ_Mode_None);
                    protocol_enqueue_rt_command(mpg_select);
                    break;
#endif
                    case PinGroup_AuxInput:
                        ioports_event(&gpio0_signals[i]);
                        break;

#if I2C_STROBE_ENABLE
                    case PinGroup_Keypad:
                        if(i2c_strobe.callback)
                            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
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

#if MPG_MODE == 1
                case PinGroup_MPG:
                    gpio_int_enable(&gpio2_signals[i], IRQ_Mode_None);
                    protocol_enqueue_rt_command(mpg_select);
                    break;
#endif
                    case PinGroup_AuxInput:
                        ioports_event(&gpio2_signals[i]);
                        break;

#if KEYPAD_ENABLE
                    case PinGroup_Keypad:
                        if(i2c_strobe.callback)
                            i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
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

    if(grp & (PinGroup_Limit|PinGroup_LimitMax))
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
    if(limits_poll) {
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
