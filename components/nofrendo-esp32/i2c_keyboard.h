#ifndef _I2C_KEYBOARD_H_
#define _I2C_KEYBOARD_H_

#include <stdio.h>
#include "driver/i2c.h"

void i2c_keyboard_master_init();
uint8_t i2c_keyboard_read();

#endif
