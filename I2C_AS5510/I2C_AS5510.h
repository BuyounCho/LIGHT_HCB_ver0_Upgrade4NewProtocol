#ifndef _I2C_AS5510_H_
#define _I2C_AS5510_H_

#include "mbed.h"
#include "setting.h"

void look_for_hardware_i2c();
void init_as5510(int i2c_address);
int offset_comp(int i2c_address);
void read_field(int i2c_address);

#endif
