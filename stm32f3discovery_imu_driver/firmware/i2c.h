#ifndef I2C_H
#define I2C_H

#include "protocol.h"

void i2c_setup(void);
void i2c_read_imu(stm32f3discovery_imu_driver::GetIMUDataResponse &resp);

#endif
