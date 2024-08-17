/*
  i2c.h - I2C support for EEPROM, keypad and Trinamic plugins

  Part of grblHAL driver for NXP LPC176x

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

#ifndef __I2C_DRIVER_H__
#define __I2C_DRIVER_H__

#include "driver.h"

#if I2C_ENABLE

#include "grbl/plugins.h"

void i2c_init (void);
void i2c_write (uint8_t addr, uint8_t *data, uint8_t len);

#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C

#include "trinamic\trinamic.h"
#include "trinamic\tmc_i2c_interface.h"

#define I2C_ADR_I2CBRIDGE 0x47

#endif

#endif // I2C_ENABLE
#endif // __I2C_DRIVER_H__
