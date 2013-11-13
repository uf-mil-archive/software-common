#include <libopencm3/stm32/f3/i2c.h>
#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <stm32f3discovery_imu_driver/protocol.h>

#include "i2c.h"

using namespace stm32f3discovery_imu_driver;

static uint8_t const I2C_ACC_ADDR = 0x19;
static uint8_t const I2C_MAG_ADDR = 0x1E;
static uint8_t const ACC_STATUS = 0x27;
static uint8_t const ACC_CTRL_REG1_A = 0x20;
static uint8_t const ACC_CTRL_REG1_A_ODR_SHIFT = 4;
static uint8_t const ACC_CTRL_REG1_A_ODR_MASK = 0xF;
static uint8_t const ACC_CTRL_REG1_A_XEN = 1 << 0;
static uint8_t const ACC_CTRL_REG4_A = 0x23;
static uint8_t const ACC_OUT_X_L_A = 0x28;
static uint8_t const ACC_OUT_X_H_A = 0x29;

void i2c_setup(void) {
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C1EN);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPBEN);
  rcc_set_i2c_clock_hsi(I2C1);

  i2c_reset(I2C1);
  /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
  gpio_set_af(GPIOB, GPIO_AF4, GPIO6| GPIO7);
  i2c_peripheral_disable(I2C1);
  //configure ANFOFF DNF[3:0] in CR1
  i2c_enable_analog_filter(I2C1);
  i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
  //Configure PRESC[3:0] SDADEL[3:0] SCLDEL[3:0]  SCLH[7:0] SCLL[7:0] in TIMINGR
  i2c_100khz_i2cclk8mhz(I2C1);
  //configure No-Stretch CR1 (only relevant in slave mode)
  i2c_enable_stretching(I2C1);
  //addressing mode
  i2c_set_7bit_addr_mode(I2C1);
  i2c_peripheral_enable(I2C1);
  
  /*uint8_t data[1]={(0x4 << ACC_CTRL_REG1_A_ODR_SHIFT) | ACC_CTRL_REG1_A_XEN};*/
  uint8_t data[1]={0x97};
  write_i2c(I2C1, I2C_ACC_ADDR, ACC_CTRL_REG1_A, 1, data);
  data[0]=0x08;
  write_i2c(I2C1, I2C_ACC_ADDR, ACC_CTRL_REG4_A, 1, data);
}

void i2c_read_imu(GetIMUDataResponse &resp) {
  static double const g0 = 9.80665;
  
  uint8_t data[6]; read_i2c(I2C1, I2C_ACC_ADDR, ACC_OUT_X_L_A|0x80, sizeof(data), data);

  for(int axis = 0; axis < 3; axis++) {
    resp.linear_acceleration[axis] = g0/1000/16 * static_cast<int16_t>((data[2*axis+1] << 8) | data[2*axis+0]);
  }
}
