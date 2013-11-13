#ifndef _BEZPKWLJADSOEQBW_
#define _BEZPKWLJADSOEQBW_

#include <stdint.h>

namespace arm_bootloader {


enum class CommandID : uint16_t {
  // commands that everything should implement
  Reset = 0,
  GetStatus,
  
  // commands specific to this firmware
  GetProgramHash = 0xf9c3,
  FlashPage,
  RunProgram,
};


struct __attribute__((packed)) ResetCommand {
};
struct __attribute__((packed)) ResetResponse {
};

struct __attribute__((packed)) GetStatusCommand {
};
struct __attribute__((packed)) GetStatusResponse {
  uint64_t bootloader_magic; static uint64_t const BOOTLOADER_MAGIC_VALUE = 0x45f5488f98180f73;
};


struct __attribute__((packed)) GetProgramHashCommand {
  uint32_t length;
};
struct __attribute__((packed)) GetProgramHashResponse {
  uint8_t error_number; // 0 means success
  uint8_t hash[32];
};

struct __attribute__((packed)) FlashPageCommand {
  uint32_t page_number;
  uint8_t page_contents[2048];
};
struct __attribute__((packed)) FlashPageResponse {
  uint8_t error_number; // 0 means success
};

struct __attribute__((packed)) RunProgramCommand {
};
struct __attribute__((packed)) RunProgramResponse {
};

typedef uint16_t ID;

struct __attribute__((packed)) Command {
  uint16_t dest;
  ID id; // 0 means don't send a response
  CommandID command;
  union {
    ResetCommand Reset;
    GetStatusCommand GetStatus;
    GetProgramHashCommand GetProgramHash;
    FlashPageCommand FlashPage;
    RunProgramCommand RunProgram;
  } args;
};

struct __attribute__((packed)) Response {
  ID id;
  union {
    ResetResponse Reset;
    GetStatusResponse GetStatus;
    GetProgramHashResponse GetProgramHash;
    FlashPageResponse FlashPage;
    RunProgramResponse RunProgram;
  } resp;
};


}

#endif
