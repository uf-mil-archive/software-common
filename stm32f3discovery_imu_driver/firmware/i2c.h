#ifndef _BRMYSZLJUPSYELNW_
#define _BRMYSZLJUPSYELNW_

#include <stm32f3discovery_imu_driver/protocol.h>

void i2c_setup(void);
void i2c_read_imu(stm32f3discovery_imu_driver::GetIMUDataResponse &resp);

#endif
