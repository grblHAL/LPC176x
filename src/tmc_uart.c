/*
 tmc_uart.c - driver code for NXP LPC176x ARM processors

 Part of grblHAL

 Copyright (c) 2021-2022  Dimitris Zervas, Terje Io

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

#include "driver.h"

#if TRINAMIC_UART_ENABLE

#include "chip.h"
#include "trinamic/common.h"
#include "trinamic/tmc2209.h"
#include "grbl/protocol.h"

#define SWS_BAUDRATE            100000      // 10us bit period
#define START_DELAY             70          // delay in us * timer clock freq
#define ABORT_TIMEOUT           10          // ms
#define TWELVE_BIT_TIMES        1           // in ms rounded up (1 is smallest we can go)
#define HALFDUPLEX_SWITCH_DELAY 4           // defined in bit-periods
#define STOP_BIT                (1<<9)      // Stop bit position in output shift "register".
#define RCV_BUF_SIZE            32          // read packet is 8 bytes

#define SUART_TIM 3
#define SUART_TIMER             timer(SUART_TIM)
#define SUART_TIMER_INT0        timerINT0(SUART_TIM)
#define SUART_TIMER_PCLK        timerPCLK(SUART_TIM)
#define SUART_IRQHandler        timerISR(SUART_TIM)

typedef struct {
    LPC_GPIO_T *port;
    uint32_t bit;
} tmc_uart_t;

typedef struct {
    volatile bool busy;
    volatile uint_fast16_t data;
} tmc_uart_tx_buffer_t;

typedef struct {
    volatile uint_fast16_t head;
    volatile uint_fast16_t tail;
    volatile int_fast16_t bit_count;
    volatile uint_fast16_t irq_count;
    bool overflow;
    bool busy;
    uint8_t data[RCV_BUF_SIZE];
} tmc_uart_rx_buffer_t;

static uint32_t period_div_2;
static tmc_uart_tx_buffer_t tx_buf;
static tmc_uart_rx_buffer_t rx_buf;
static tmc_uart_t uart[TMC_N_MOTORS_MAX], *active_uart;

// some defines until added to core
#define PinGroup_MotorStandBy   0
#define Output_StandByX         0
#define Output_StandByY         1
#define Output_StandByZ         2
#define Output_StandByM3        3
#define Output_StandByM4        4

inline static uint8_t gpio_to_pn (LPC_GPIO_T *port)
{
    return ((uint32_t)port - LPC_GPIO0_BASE) / sizeof(LPC_GPIO_T);
}

static output_signal_t outputpin[] = {
#ifdef X_STANDBY
    { .id = Output_StandByX,           .port = X_STANDBY_PORT,   .pin = X_STANDBY_PIN,     .group = PinGroup_MotorStandBy },
#endif
#ifdef Y_STANDBY
    { .id = Output_StandByY,           .port = Y_STANDBY_PORT,   .pin = Y_STANDBY_PIN,     .group = PinGroup_MotorStandBy },
#endif
#ifdef Z_STANDBY
    { .id = Output_StandByZ,           .port = Z_STANDBY_PORT,   .pin = Z_STANDBY_PIN,     .group = PinGroup_MotorStandBy },
#endif
#if defined(M3_AVAILABLE) && defined(M3_STANDBY)
    { .id = Output_StandByM3,          .port = M3_STANDBY_PORT,  .pin = M3_STANDBY_PIN,     .group = PinGroup_MotorStandBy },
#endif
#if defined(M4_AVAILABLE) && defined(M4_STANDBY)
    { .id = Output_StandByM4,          .port = M4_STANDBY_PORT,  .pin = M4_STANDBY_PIN,     .group = PinGroup_MotorStandBy },
#endif
};

static inline void setTX (void)
{
    active_uart->port->DIR |= active_uart->bit;
}

static inline void setRX (void)
{
    active_uart->port->DIR &= ~active_uart->bit;
}

/**
 * @brief Software Serial send byte
 * @param None
 * @retval None
 * This is called by the interrupt handler.
 */
static inline void send (void)
{
    if(tx_buf.data) {
        DIGITAL_OUT(active_uart->port, active_uart->bit, tx_buf.data & 1);  // drive bit out
        tx_buf.data >>= 1;                                                  // shift to next bit
    } else
        tx_buf.busy = false;
}

static void write_n (uint8_t data[], uint32_t length)
{
    uint_fast8_t i = 0;

    setTX();
    while (tx_buf.busy)                             // should not be anything pending but...
        protocol_execute_noop(0);

    tx_buf.data = (data[i++] << 1) | STOP_BIT;      // form first word with START and STOP bits
    tx_buf.busy = true;
    length--;

    SUART_TIMER->TC = 0;
    SUART_TIMER->TCR = 0b01;

    while (tx_buf.busy)
        protocol_execute_noop(0);

    do {
        tx_buf.data = (data[i++] << 1) | STOP_BIT;  // form next word
        tx_buf.busy = true;
        while(tx_buf.busy)                         // wait for byte to finish
            protocol_execute_noop(0);
    } while(--length);
/*
    tx_buf.busy = true;
    while (tx_buf.busy)
        protocol_execute_noop(0);
*/
    SUART_TIMER->TCR = 0;
}

static int16_t read_byte (void)
{
    if (rx_buf.tail == rx_buf.head)
        return -1;

    int16_t byte = (int16_t)rx_buf.data[rx_buf.tail];   // get next byte
    rx_buf.tail = BUFNEXT(rx_buf.tail, rx_buf);

    return byte;
}

/**
 * @brief Software Serial receive byte
 * @param None
 * @retval None
 * This is called by the interrupt handler.
 * Tihs is only called if rx_busy == true;
 */
static inline void rcv (void)
{
    static volatile uint32_t rx_byte;

    bool inbit = DIGITAL_IN(active_uart->port, active_uart->bit);

    hal.coolant.set_state((coolant_state_t){1});

    if (rx_buf.bit_count == -1) {                               // -1 means waiting for START (0)
        if (!inbit) {
            rx_buf.bit_count = 0;                               // 0: START bit received (low)
            rx_byte = 0;
        }
    } else if (rx_buf.bit_count > 7) {                          // > 7 means waiting for STOP (high)
        if (inbit) {
            uint_fast16_t next = BUFNEXT(rx_buf.head, rx_buf);
            if (next != rx_buf.tail) {                          // room in buffer?
                rx_buf.data[rx_buf.head] = rx_byte;             // save new byte
                rx_buf.head = next;
            } else {                                            // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
                rx_buf.overflow = true;
            }
        }
        rx_buf.bit_count = -1;                                  // wait for next START
    } else {
        rx_byte >>= 1;                                          // shift previous
        if (inbit) {
            rx_byte |= 0x80;                                    // OR in new
        }
        rx_buf.bit_count++;                                     // Preprare for next bit
    }

    hal.coolant.set_state((coolant_state_t){0});
}

static void stop_listening (void)
{
    uint8_t count = rx_buf.irq_count;

    // Need to wait HALFDUPLEX_SWITCH_DELAY sample periods for TMC2209 to release it's transmitter
    /*
     * Some Notes are in order here:
     * There are 20K pulldowns on the PDN_UART pin on the stepper driver board.
     * These pulldowns are small enough to overpower the pull-up on the input to
     * the STM32, and this is enough to cause a glitch as defined by TMC (<16 clocks).
     * So, we know the TMC is driving a 1 since we detected a STOP already, we just
     * want to turn our driver back on just before it turns its driver off so we don't
     * get a glitch.  This is one advantage of the FYSETC stepper drivers over
     * the BTT stepper drivers, they have 1K in series with the PDN_UART pin.
     * Either way, we are not really fighting the PDN_UART pin since both
     * the TMC2209 chip and the STM32 are driving high.
     */
    while ((rx_buf.irq_count + 256 - count) % 256 < HALFDUPLEX_SWITCH_DELAY)
        protocol_execute_noop(0);

    setTX();        // turn driver on just before the TMC driver goes off

    // Wait one more bit period without which the TMC won't respond
    while ((rx_buf.irq_count + 256 - count) % 256 < (HALFDUPLEX_SWITCH_DELAY + 1))
        protocol_execute_noop(0);

    // Now we can wrap things up
    rx_buf.busy = false;
    SUART_TIMER->TCR = 0; // Disable output event
}

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *rdgr)
{
    static TMC_uart_write_datagram_t wdgr = { 0 };
    static TMC_uart_write_datagram_t bad = { 0 };

    // Remember the port and pin we are using
    active_uart = &uart[driver.id];

    // purge anything in buffer
    rx_buf.tail = rx_buf.head = 0;

    write_n(rdgr->data, sizeof(TMC_uart_read_datagram_t)); // send read request

    setRX();

    // Look for START (0)
    // If read request had CRC error or some other issue, we won't get a reply
    // so we need to timeout
    uint32_t ms = hal.get_elapsed_ticks();

    while (DIGITAL_IN(active_uart->port, active_uart->bit)) {
        if (hal.get_elapsed_ticks() - ms > ABORT_TIMEOUT) {
            hal.delay_ms(TWELVE_BIT_TIMES + 1, NULL);
            setTX();                // turn on our driver
            return &bad;            // return {0}
        }
    }

    rx_buf.busy = true;
    rx_buf.bit_count = -1; // look for START bit

    SUART_TIMER->TC = period_div_2 + START_DELAY;
    SUART_TIMER->TCR = 0b01;

    // Wait for read response
    int16_t res;
    ms = hal.get_elapsed_ticks();

    for (uint8_t i = 0; i < 8;) {
        if (hal.get_elapsed_ticks() - ms > ABORT_TIMEOUT) {
            rx_buf.busy = false;
            SUART_TIMER->TCR = 0;
            hal.delay_ms(TWELVE_BIT_TIMES + 1, NULL);
            setTX();
            return &bad;
        }

        if ((res = read_byte()) != -1)
            wdgr.data[i++] = res;
    }

    // purge anything left in buffer
    rx_buf.tail = rx_buf.head = 0;

    stop_listening();

    return &wdgr;
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    // Remember the port and pin we are using
    active_uart = &uart[driver.id];

    write_n(dgr->data, sizeof(TMC_uart_write_datagram_t));
}

/**
 * @brief Software Serial Read Byte
 * @param None
 * @retval Byte read
 *
 * Returns the next byte from the receive buffer or -1 on underflow.
 */

static void add_uart_pin (xbar_t *gpio, void *data)
{
    if (gpio->group == PinGroup_MotorUART)
      switch (gpio->function) {

        case Bidirectional_MotorUARTX:
            uart[X_AXIS].port = (LPC_GPIO_T*) gpio->port;
            uart[X_AXIS].bit = 1U << gpio->pin;
            break;

        case Bidirectional_MotorUARTY:
            uart[Y_AXIS].port = (LPC_GPIO_T*) gpio->port;
            uart[Y_AXIS].bit = 1U << gpio->pin;
            break;

        case Bidirectional_MotorUARTZ:
            uart[Z_AXIS].port = (LPC_GPIO_T*) gpio->port;
            uart[Z_AXIS].bit = 1U << gpio->pin;
            break;

        case Bidirectional_MotorUARTM3:
            uart[3].port = (LPC_GPIO_T*) gpio->port;
            uart[3].bit = 1U << gpio->pin;
            break;

        case Bidirectional_MotorUARTM4:
            uart[4].port = (LPC_GPIO_T*) gpio->port;
			uart[4].bit = 1U << gpio->pin;
            break;

        default:
            break;
    }
}

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    static bool init_ok = false;

    UNUSED(motors);

    if (!init_ok) {

        Chip_TIMER_Init(SUART_TIMER);

        SUART_TIMER->TCR = 0;           // disable
        SUART_TIMER->CTCR = 0;          // timer mode
        SUART_TIMER->PR = 0;            // set prescale
        SUART_TIMER->MCR = MR0I | MR0R; // MR0: !stop, reset, interrupt
        SUART_TIMER->CCR = 0;           // no capture
        SUART_TIMER->EMR = 0;           // no external match
        SUART_TIMER->MR[0] = (SystemCoreClock / Chip_Clock_GetPCLKDiv(SUART_TIMER_PCLK) / SWS_BAUDRATE) & 0xFFFFFFFE;

        period_div_2 = SUART_TIMER->MR[0] >> 1;

        NVIC_EnableIRQ(SUART_TIMER_INT0);       // Enable software UART interrupt
        NVIC_SetPriority(SUART_TIMER_INT0, 1);  // Same with stepper timer - no idea why

        hal.enumerate_pins(true, add_uart_pin, NULL);
    }
}

void driver_preinit (motor_map_t motor, trinamic_driver_config_t *config)
{
    config->address = 0;
}

void tmc_uart_init (void)
{
    static trinamic_driver_if_t driver_if = {
        .on_drivers_init = if_init,
        .on_driver_preinit = driver_preinit
    };

    trinamic_if_init(&driver_if);
}

void SUART_IRQHandler (void)
{
    SUART_TIMER->IR = SUART_TIMER->IR;

    if (tx_buf.busy)
        send();
    if (rx_buf.busy) {
        rcv();
        rx_buf.irq_count++;
    }
}

#endif

