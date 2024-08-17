/*
  i2c.c - I2C support

  Part of grblHAL driver for NXP LPC176x

  Copyright (c) 2019-2024 Terje Io

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

#include "i2c.h"

#if I2C_ENABLE

#include <string.h>

#include "grbl/hal.h"

#define MAX_PAGE_SIZE 64

void i2c_init (void)
{

#if I2C_ENABLE == 1
    Chip_IOCON_PinMux(LPC_IOCON, 0, 19, IOCON_MODE_INACT, IOCON_FUNC3);
    Chip_IOCON_PinMux(LPC_IOCON, 0, 20, IOCON_MODE_INACT, IOCON_FUNC3);
    Chip_IOCON_EnableOD(LPC_IOCON, 0, 19);
    Chip_IOCON_EnableOD(LPC_IOCON, 0, 20);

    static const periph_pin_t scl = {
        .function = Output_SCK,
        .group = PinGroup_I2C,
        .port = LPC_GPIO0,
        .pin = 20,
//        .mode = { .mask = PINMODE_OD }
    };

    static const periph_pin_t sda = {
        .function = Bidirectional_SDA,
        .group = PinGroup_I2C,
        .port = LPC_GPIO0,
        .pin = 19,
        //        .mode = { .mask = PINMODE_OD }
    };
#else
    Chip_IOCON_PinMux(LPC_IOCON, 0, 0, IOCON_MODE_INACT, IOCON_FUNC3);
    Chip_IOCON_PinMux(LPC_IOCON, 0, 1, IOCON_MODE_INACT, IOCON_FUNC3);
    Chip_IOCON_EnableOD(LPC_IOCON, 0, 0);
    Chip_IOCON_EnableOD(LPC_IOCON, 0, 1);

    static const periph_pin_t scl = {
        .function = Output_SCK,
        .group = PinGroup_I2C,
        .port = LPC_GPIO0,
        .pin = 1,
        //        .mode = { .mask = PINMODE_OD }
    };

    static const periph_pin_t sda = {
        .function = Bidirectional_SDA,
        .group = PinGroup_I2C,
        .port = LPC_GPIO0,
        .pin = 0,
        //        .mode = { .mask = PINMODE_OD }
    };
#endif

    Chip_I2C_Init(I2C1);
    Chip_I2C_SetClockRate(I2C1, 100000);

    Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);

    hal.periph_port.register_pin(&scl);
    hal.periph_port.register_pin(&sda);
}

#if EEPROM_ENABLE

nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *i2c, bool read)
{
    static I2C_XFER_T xfer;
    static uint8_t txbuf[MAX_PAGE_SIZE + 2];

    if(i2c->word_addr_bytes == 2) {
        txbuf[0] = i2c->word_addr >> 8;
        txbuf[1] = i2c->word_addr & 0xFF;
    } else
        txbuf[0] = i2c->word_addr;

    xfer.slaveAddr = i2c->address;
    xfer.rxSz = read ? i2c->count : 0;
    xfer.rxBuff = read ? i2c->data : NULL;
    xfer.txBuff = txbuf;

    if(!read) {
        xfer.txSz = i2c->word_addr_bytes + i2c->count;
        memcpy(&txbuf[i2c->word_addr_bytes], i2c->data, i2c->count);
    } else
        xfer.txSz = i2c->word_addr_bytes;

    Chip_I2C_MasterTransfer(I2C1, &xfer);

#if !EEPROM_IS_FRAM
    if(!read)
        hal.delay_ms(15, NULL);
#endif

    return NVS_TransferResult_OK;
}

#endif

void i2c_write (uint8_t addr, uint8_t *data, uint8_t len)
{
    static I2C_XFER_T xfer;

    xfer.slaveAddr = addr;
    xfer.rxSz = 0;
    xfer.rxBuff = NULL;
    xfer.txBuff = data;
    xfer.txSz = len;

    Chip_I2C_MasterTransfer(I2C1, &xfer);
}

#if KEYPAD_ENABLE == 1

void i2c_get_keycode (uint_fast16_t addr, keycode_callback_ptr callback)
{
    uint8_t keycode;

    // This should be changed to be asynchronous!
    if(Chip_I2C_MasterRead(I2C1, (uint8_t)addr, &keycode, 1) == 1)
        callback(keycode);
}

#endif

#endif // I2C_ENABLE
