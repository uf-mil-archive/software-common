#ifndef ARM_TEST_STM32F3DISCOVERY_IMU_DRIVER_FIRMWARE_PROTOCOL_H
#define ARM_TEST_STM32F3DISCOVERY_IMU_DRIVER_FIRMWARE_PROTOCOL_H

#include <stdint.h>

namespace stm32f3discovery_imu_driver {


enum class CommandID : uint16_t {
  // commands that everything should implement
  Reset = 0,
  GetStatus,
  
  // commands specific to this firmware
  GetIMUData = 0x6393,
};


struct __attribute__((packed)) ResetCommand {
};
struct __attribute__((packed)) ResetResponse {
};

struct __attribute__((packed)) GetStatusCommand {
};
struct __attribute__((packed)) GetStatusResponse {
  uint64_t magic; static uint64_t const MAGIC_VALUE = 0xb67f739fff9612cd;
};


struct __attribute__((packed)) GetIMUDataCommand {
};
struct __attribute__((packed)) GetIMUDataResponse {
  double linear_acceleration[3];
};


typedef uint16_t ID;

struct __attribute__((packed)) Command {
  uint16_t dest;
  ID id; // 0 means don't send a response
  CommandID command;
  union {
    ResetCommand Reset;
    GetStatusCommand GetStatus;
    GetIMUDataCommand GetIMUData;
  } args;
};

struct __attribute__((packed)) Response {
  ID id;
  union {
    ResetResponse Reset;
    GetStatusResponse GetStatus;
    GetIMUDataResponse GetIMUData;
  } resp;
};


}

#endif
