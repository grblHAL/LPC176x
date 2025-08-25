/*
  btt_skr_1.x.c - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#if defined(BOARD_BTT_SKR_13) || defined(BOARD_BTT_SKR_14_TURBO)

#include "chip.h"

#if TRINAMIC_SPI_ENABLE

#include "trinamic/common.h"

#define TRINAMIC_MOSI_BIT (1<<TRINAMIC_MOSI_PIN)
#define TRINAMIC_MISO_BIT (1<<TRINAMIC_MISO_PIN)
#define TRINAMIC_SCK_BIT (1<<TRINAMIC_SCK_PIN)

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static struct {
    LPC_GPIO_T *port;
    uint32_t bit;
} cs[TMC_N_MOTORS_MAX];

inline static void delay (void)
{
    volatile uint32_t dly = 10;

    while(--dly)
        __ASM volatile ("nop");
}

static uint8_t sw_spi_xfer (uint8_t byte)
{
    uint_fast8_t msk = 0x80, res = 0;

    TRINAMIC_SCK_PORT->CLR = TRINAMIC_SCK_BIT;

    do {
        DIGITAL_OUT(TRINAMIC_MOSI_PORT, TRINAMIC_MOSI_BIT, byte & msk);
        msk >>= 1;
        delay();
        res = (res << 1) | DIGITAL_IN(TRINAMIC_MISO_PORT, TRINAMIC_MISO_BIT);
        TRINAMIC_SCK_PORT->SET = TRINAMIC_SCK_BIT;
        delay();
        if(msk)
            TRINAMIC_SCK_PORT->CLR = TRINAMIC_SCK_BIT;
    } while (msk);

    return (uint8_t)res;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    cs[driver.id].port->CLR = cs[driver.id].bit;

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    cs[driver.id].port->SET = cs[driver.id].bit;
    delay();
    cs[driver.id].port->CLR = cs[driver.id].bit;

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    cs[driver.id].port->SET = cs[driver.id].bit;

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    cs[driver.id].port->CLR = cs[driver.id].bit;

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    cs[driver.id].port->SET = cs[driver.id].bit;

    return status;
}


static void add_cs_pin (xbar_t *gpio, void *data)
{
    if(gpio->group == PinGroup_MotorChipSelect) {
        switch(gpio->function) {

            case Output_MotorChipSelectX:
                cs[X_AXIS].bit = 1U << gpio->pin;
                cs[X_AXIS].port = (LPC_GPIO_T *)gpio->port;
                break;
            case Output_MotorChipSelectY:
                cs[Y_AXIS].bit = 1U << gpio->pin;
                cs[Y_AXIS].port = (LPC_GPIO_T *)gpio->port;
                break;
            case Output_MotorChipSelectZ:
                cs[Z_AXIS].bit = 1U << gpio->pin;
                cs[Z_AXIS].port = (LPC_GPIO_T *)gpio->port;
                break;
            case Output_MotorChipSelectM3:
                cs[3].bit = 1U << gpio->pin;
                cs[3].port = (LPC_GPIO_T *)gpio->port;
                break;
            case Output_MotorChipSelectM4:
                cs[4].bit = 1U << gpio->pin;
                cs[4].port = (LPC_GPIO_T *)gpio->port;
                break;
            case Output_MotorChipSelectM5:
                cs[5].bit = 1U << gpio->pin;
                cs[5].port = (LPC_GPIO_T *)gpio->port;
                break;

            default:
                break;
        }
    }
}

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    static bool init_ok = false;

    UNUSED(motors);

    if(!init_ok) {

        init_ok = true;

        Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MOSI_PN, TRINAMIC_MOSI_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
        Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MISO_PN, TRINAMIC_MISO_PIN, IOCON_MODE_PULLUP, IOCON_FUNC0);
        Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_SCK_PN, TRINAMIC_SCK_PIN, IOCON_MODE_INACT, IOCON_FUNC0);

        Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MOSI_PN, TRINAMIC_MOSI_PIN);
        Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MISO_PN, TRINAMIC_MOSI_PIN);
        Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_SCK_PN, TRINAMIC_MOSI_PIN);

        TRINAMIC_MOSI_PORT->DIR |= TRINAMIC_MOSI_BIT;
        TRINAMIC_MISO_PORT->DIR &= ~TRINAMIC_MISO_BIT;

        TRINAMIC_SCK_PORT->DIR |= TRINAMIC_SCK_BIT;
        TRINAMIC_SCK_PORT->SET = TRINAMIC_SCK_BIT;

        hal.enumerate_pins(true, add_cs_pin, NULL);
    }
}

#endif

void board_init (void)
{
#if TRINAMIC_SPI_ENABLE

    static trinamic_driver_if_t driver_if = {
        .on_drivers_init = if_init
    };

    trinamic_if_init(&driver_if);
#elif TRINAMIC_UART_ENABLE
    extern void tmc_uart_init (void);
    tmc_uart_init();
#endif
}

#endif

