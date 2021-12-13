/*
 btt_skr_e3_turbo.c - driver code for NXP LPC176x ARM processors

 Part of grblHAL

 Copyright (c) 2021  Dimitris Zervas, Terje Io

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

#if defined(BOARD_BTT_SKR_E3_TURBO)

#include "chip.h"
#include "trinamic/common.h"
#include "trinamic/tmc2209.h"

#define SWS_BAUDRATE            100000       // 10us bit period
#define START_DELAY             46          // delay in us * timer clock freq
#define ABORT_TIMEOUT           10           // ms
#define TWELVE_BIT_TIMES        1           // in ms rounded up (1 is smallest we can go)
#define HALFDUPLEX_SWITCH_DELAY 4           // defined in bit-periods
#define RCV_BUF_SIZE            32          // read packet is 8 bytes

#define SUART_TIM 3
#define SUART_TIMER             timer(SUART_TIM)
#define SUART_TIMER_INT0        timerINT0(SUART_TIM)
#define SUART_TIMER_PCLK        timerPCLK(SUART_TIM)
#define SUART_IRQHandler        timerISR(SUART_TIM)

static LPC_GPIO_T *port;
static uint32_t bit;
static uint32_t period_div_2;
static volatile uint32_t tx_buffer;
static volatile int32_t tx_bit_count;
static volatile uint32_t rx_buffer;
static volatile int32_t rx_bit_count;
static volatile bool rx_busy = false;
static volatile bool tx_busy = false;

static volatile uint8_t receive_buffer[RCV_BUF_SIZE];
static volatile uint8_t wr_ptr = 0;
static volatile uint8_t rd_ptr = 0;
static bool buffer_overflow = false;
static volatile uint8_t rx_irq_count = 0;

static struct {
	LPC_GPIO_T *port;
	uint32_t bit;
} uart[TMC_N_MOTORS_MAX];

void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr);
static void write_n(uint8_t data[], uint32_t length);
static void send(void);
TMC_uart_write_datagram_t* tmc_uart_read(trinamic_motor_t driver,
		TMC_uart_read_datagram_t *rdgr);
static int32_t rx_buffer_count(void);
static int32_t read_byte(void);
static void rcv(void);
static void stop_listening(void);
static inline void setTX();
static inline void setRX();
static void TIMER_Init(void);
void SUART_IRQHandler(void);

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

void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
	// Remember the port and pin we are using
	port = uart[driver.id].port;
	bit = uart[driver.id].bit;

	write_n(dgr->data, sizeof(TMC_uart_write_datagram_t));
}

static void write_n(uint8_t data[], uint32_t length)
{
	uint8_t i;

	setTX();
	while (tx_busy);                    // should not be anything pending but...

	for (i = 0; i < length; i++) {
		tx_buffer = data[i];             // form next word
		tx_bit_count = -1;
		tx_busy = true;

		if (i == 0) {
			SUART_TIMER->TC = START_DELAY;
			SUART_TIMER->TCR = 0b01;
		}

		while (tx_busy);                  // wait for byte to finish
	}
	tx_busy = true;
	while (tx_busy);

	SUART_TIMER->TCR = 0;
}

/**
 * @brief Software Serial send byte
 * @param None
 * @retval None
 * This is called by the interrupt handler.
 */
static void send(void)
{
	if (tx_bit_count == -1) {
		DIGITAL_OUT(port, bit, 0);              // START bit (0)
	} else if (tx_bit_count < 8) {
		DIGITAL_OUT(port, bit, tx_buffer & 1);  // drive bit out
		tx_buffer >>= 1;                        // shift to next bit
	} else {
		DIGITAL_OUT(port, bit, 1);              // STOP bit (1)
		tx_busy = false;                        // we are done with STOP bit
	}

	tx_bit_count++;
}

TMC_uart_write_datagram_t* tmc_uart_read(trinamic_motor_t driver, TMC_uart_read_datagram_t *rdgr)
{
	static TMC_uart_write_datagram_t wdgr = { 0 };
	static TMC_uart_write_datagram_t bad = { 0 };

	// Remember the port and pin we are using
	port = uart[driver.id].port;
	bit = uart[driver.id].bit;

	// purge anything in buffer
	while (rx_buffer_count())
		read_byte();

	write_n(rdgr->data, sizeof(TMC_uart_read_datagram_t)); // send read request

	setRX();

	// Look for START (0)
	// If read request had CRC error or some other issue, we won't get a reply
	// so we need to timeout
	uint8_t timeout = ABORT_TIMEOUT;
	uint32_t ms = hal.get_elapsed_ticks();
	bool inbit;
	do {
		uint32_t ms2 = hal.get_elapsed_ticks();
		if (ms2 != ms) {
			ms = ms2;
			if (--timeout == 0) {
				setTX();                  // turn on our driver
				return &bad;              // return {0}
			}
		}
		inbit = DIGITAL_IN(port, bit);
	} while (inbit);

	rx_busy = true;
	rx_bit_count = -1; // look for START bit

	SUART_TIMER->TC = period_div_2;
	SUART_TIMER->TCR = 0b01;

	// Wait for read response
	timeout = ABORT_TIMEOUT;
	ms = hal.get_elapsed_ticks();
	for (uint8_t i = 0; i < 8;) {
		uint32_t ms2 = hal.get_elapsed_ticks();
		if (ms2 != ms) {
			if (--timeout == 0) {
				rx_busy = false;
				SUART_TIMER->TCR = 0;
				setTX();

				return &bad;
			}
			ms = ms2;
		}

		int16_t res = read_byte();
		if (res != -1) {
			wdgr.data[i] = res;
			i++;
		}
	}

	// purge anything left in buffer
	while (rx_buffer_count())
		read_byte();

	stop_listening();
	return &wdgr;
}

/**
 * @brief Get Read Buffer Count
 * @param None
 * @returnval Count
 */
static int32_t rx_buffer_count(void)
{
	return (rd_ptr + RCV_BUF_SIZE - wr_ptr) % RCV_BUF_SIZE;
}

/**
 * @brief Software Serial Read Byte
 * @param None
 * @retval Byte read
 *
 * Returns the next byte from the receive buffer or -1 on underflow.
 */
static int32_t read_byte(void)
{
	if (rd_ptr == wr_ptr) {
		return -1;
	}

	uint8_t byte = receive_buffer[rd_ptr]; // get next byte
	rd_ptr = (rd_ptr + 1) % RCV_BUF_SIZE;
	return byte;
}

/**
 * @brief Software Serial receive byte
 * @param None
 * @retval None
 * This is called by the interrupt handler.
 * Tihs is only called if rx_busy == true;
 */
static void rcv(void)
{
	bool inbit = DIGITAL_IN(port, bit);

	if (rx_bit_count == -1) {        // -1 means waiting for START (0)
		if (!inbit) {
			rx_bit_count = 0;           // 0: START bit received (low)
			rx_buffer = 0;
		}
	} else if (rx_bit_count > 7) {  // > 7 means waiting for STOP (high)
		if (inbit) {
			uint8_t next = (wr_ptr + 1) % RCV_BUF_SIZE;
			if (next != rd_ptr) {     // room in buffer?
				receive_buffer[wr_ptr] = rx_buffer; // save new byte
				wr_ptr = next;
			} else { // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
				buffer_overflow = true;
			}
		}
		rx_bit_count = -1;          // wait for next START
	} else {
		rx_buffer >>= 1;            // shift previous
		if (inbit) {
			rx_buffer |= 0x80;        // OR in new
		}
		rx_bit_count++;             // Preprare for next bit
	}
}

static void stop_listening(void)
{
	uint8_t count = rx_irq_count;

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
    while ((rx_irq_count + 256 - count) % 256 < HALFDUPLEX_SWITCH_DELAY);

    setTX();		// turn driver on just before the TMC driver goes off

	// Wait one more bit period without which the TMC won't respond
    while ((rx_irq_count + 256 - count) % 256 < (HALFDUPLEX_SWITCH_DELAY + 1));

	// Now we can wrap things up
	rx_busy = false;
	SUART_TIMER->TCR = 0; // Disable output event
}

static inline void setTX()
{
	port->DIR |= bit;
}

static inline void setRX()
{
	port->DIR &= ~bit;
}

/**
 * @brief TIMER Initialization Function
 * @param None
 * @retval None
 */
static void TIMER_Init(void)
{
	uint32_t period;

	// Determine the period based on the timer clock
	period = (SystemCoreClock / Chip_Clock_GetPCLKDiv(SUART_TIMER_PCLK))
			/ (SystemCoreClock / Chip_Clock_GetPCLKDiv(SUART_TIMER_PCLK)) - 1;

	Chip_TIMER_Init(SUART_TIMER);

	SUART_TIMER->TCR = 0;            // disable
	SUART_TIMER->CTCR = 0;           // timer mode
	SUART_TIMER->PR = period;     // set prescale
	SUART_TIMER->MCR = MR0I | MR0R;    // MR0: !stop, reset, interrupt
	SUART_TIMER->CCR = 0;            // no capture
	SUART_TIMER->EMR = 0;            // no external match
	SUART_TIMER->MR[0] = (SystemCoreClock / Chip_Clock_GetPCLKDiv(SUART_TIMER_PCLK)) / SWS_BAUDRATE;

	period_div_2 = ((SystemCoreClock / Chip_Clock_GetPCLKDiv(SUART_TIMER_PCLK)) / SWS_BAUDRATE) / 2;

	NVIC_EnableIRQ(SUART_TIMER_INT0);  // Enable software UART interrupt

	NVIC_SetPriority(SUART_TIMER_INT0, 1); // Same with stepper timer - no idea why
}

void SUART_IRQHandler(void)
{
	SUART_TIMER->IR = SUART_TIMER->IR;

	if (tx_busy)
		send();
	if (rx_busy) {
		rcv();
		rx_irq_count++;
	}
}

static void add_uart_pin (xbar_t *gpio)
{
	if (gpio->group == PinGroup_MotorUART)
	  switch (gpio->function) {

		case Bidirectional_MotorUARTX:
			uart[X_AXIS].port = (LPC_GPIO_T*) gpio->port;
			uart[X_AXIS].bit = gpio->bit;
			break;

		case Bidirectional_MotorUARTY:
			uart[Y_AXIS].port = (LPC_GPIO_T*) gpio->port;
			uart[Y_AXIS].bit = gpio->bit;
			break;

		case Bidirectional_MotorUARTZ:
			uart[Z_AXIS].port = (LPC_GPIO_T*) gpio->port;
			uart[Z_AXIS].bit = gpio->bit;
			break;

		case Bidirectional_MotorUARTM3:
			uart[3].port = (LPC_GPIO_T*) gpio->port;
			uart[3].bit = gpio->bit;
			break;

		case Bidirectional_MotorUARTM4:
			uart[4].port = (LPC_GPIO_T*) gpio->port;
			uart[4].bit = gpio->bit;
			break;

		default:
			break;
	}
}

static void if_init(uint8_t motors, axes_signals_t enabled)
{
	static bool init_ok = false;

	UNUSED(motors);

	if (!init_ok) {
		TIMER_Init();

		hal.enumerate_pins(true, add_uart_pin);
	}
}

void driver_preinit (motor_map_t motor, trinamic_driver_config_t *config)
{
    config->address = 0;
}

void board_init (void)
{
	static trinamic_driver_if_t driver_if = {
	    .on_drivers_init = if_init,
        .on_driver_preinit = driver_preinit
	};

	trinamic_if_init(&driver_if);

    uint32_t i;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        outputpin[i].bit = 1U << outputpin[i].pin;
        // Cleanup after (potentially) sloppy bootloader
        Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, gpio_to_pn(outputpin[i].port), outputpin[i].pin, IOCON_MODE_INACT, IOCON_FUNC0);
        //
        outputpin[i].port->DIR |= outputpin[i].bit;
        outputpin[i].port->CLR = outputpin[i].bit;
    }
}

#endif

