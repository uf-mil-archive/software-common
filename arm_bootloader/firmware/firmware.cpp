#include <cstring>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/stm32/f3/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

#include "sha256.h"
#include "../include/arm_bootloader/subbus_protocol.h"

#include "protocol.h"

using namespace arm_bootloader;

__attribute__((warn_unused_result))
bool flash_erase(void *dest) {
  if(reinterpret_cast<uint32_t>(dest) % 2048) while(true);
  
  FLASH_CR |= FLASH_CR_PER;
  FLASH_AR = reinterpret_cast<uint32_t>(dest);
  FLASH_CR |= FLASH_CR_STRT;
  flash_wait_for_last_operation();
  FLASH_CR &= ~static_cast<uint32_t>(FLASH_CR_PER);
  if(!(FLASH_SR & FLASH_SR_EOP)) return false;
  flash_clear_eop_flag();
  return true;
}

__attribute__((warn_unused_result))
bool flash_write(void *dest, void *src, size_t length_in_bytes) {
  if(length_in_bytes % 2) while(true);
  
  for(uint32_t i = 0; i < length_in_bytes/2; i++) {
      FLASH_CR |= FLASH_CR_PG;
      reinterpret_cast<uint16_t*>(dest)[i] =
        reinterpret_cast<uint16_t*>(src)[i];
      flash_wait_for_last_operation();
      FLASH_CR &= ~static_cast<uint32_t>(FLASH_CR_PG);
      if(!(FLASH_SR & FLASH_SR_EOP)) return false;
      flash_clear_eop_flag();
  }
  return true;
}

extern unsigned _end_empty;
uint8_t *flash_start = reinterpret_cast<uint8_t*>(0x8000000);
uint8_t *first_used_page_start =
  reinterpret_cast<uint8_t*>(reinterpret_cast<size_t>(&_end_empty) &
    ~(static_cast<size_t>(2048)-1));
uint8_t *last_unused_page_start = first_used_page_start - 2048;

struct LastPage {
  uint32_t program_stack_pointer;
  uint32_t program_reset_vector;
};
LastPage *lastpage = reinterpret_cast<LastPage *>(last_unused_page_start);

arm_bootloader::ChecksumAdder<
  arm_bootloader::Packetizer<void (*)(uint8_t byte)> > *p_checksumadder;

void messageReceived(const Command &msg) {
  if(msg.dest != 0x1234) return;
  
  Response resp; memset(&resp, 0, sizeof(resp));
  resp.id = msg.id;
  
  switch(msg.command) {
  
    case CommandID::Reset: {
      // action happens later, after response is sent
    } break;
  
    case CommandID::GetStatus: {
      resp.resp.GetStatus.bootloader_magic =
        GetStatusResponse::BOOTLOADER_MAGIC_VALUE;
    } break;
    
    case CommandID::GetProgramHash: {
      if(flash_start + msg.args.GetProgramHash.length >
          last_unused_page_start) {
        resp.resp.GetProgramHash.error_number = 1;
        break;
      }
      
      sha256_state md; sha256_init(md);
      for(uint32_t pos = 0; pos < msg.args.GetProgramHash.length; pos++) {
        uint8_t data = flash_start[pos];
        if(pos < 4) data = *(reinterpret_cast<uint8_t*>(&lastpage->program_stack_pointer) + pos);
        else if(pos < 8) data = *(reinterpret_cast<uint8_t*>(&lastpage->program_reset_vector) + (pos - 4));
        sha256_process(md, &data, 1);
      }
      
      resp.resp.GetProgramHash.error_number = 0;
      sha256_done(md, resp.resp.GetProgramHash.hash);
    } break;
    
    case CommandID::FlashPage: {
      uint8_t *page_start = flash_start + 2048*msg.args.FlashPage.page_number;
      
      if(page_start >= last_unused_page_start || page_start < flash_start) {
        resp.resp.FlashPage.error_number = 1;
        break;
      }
      
      flash_unlock();
      flash_wait_for_last_operation();
      
      flash_clear_eop_flag();
      FLASH_CR &= ~static_cast<uint32_t>(FLASH_CR_MER) & ~static_cast<uint32_t>(FLASH_CR_PER) & ~static_cast<uint32_t>(FLASH_CR_PG); // stlink programmer seems to leave PG set
      flash_wait_for_last_operation();
      
      uint8_t page[2048]; memcpy(page, msg.args.FlashPage.page_contents, 2048);
      if(page_start == flash_start) {
        LastPage new_lastpage;
        new_lastpage.program_stack_pointer = 0;
        new_lastpage.program_reset_vector = 0;
        
        memcpy(&new_lastpage.program_stack_pointer, page+0, sizeof(new_lastpage.program_stack_pointer));
        memcpy(page+0, flash_start+0, sizeof(new_lastpage.program_stack_pointer));
        memcpy(&new_lastpage.program_reset_vector, page+4, sizeof(new_lastpage.program_reset_vector));
        memcpy(page+4, flash_start+4, sizeof(new_lastpage.program_reset_vector));
        
        if(!flash_erase(last_unused_page_start)) {
          resp.resp.FlashPage.error_number = 2;
          break;
        }
        if(!flash_write(lastpage, &new_lastpage, sizeof(new_lastpage))) {
          resp.resp.FlashPage.error_number = 3;
          break;
        }
      }
      
      if(!flash_erase(page_start)) {
        resp.resp.FlashPage.error_number = 4;
        break;
      }
      
      if(!flash_write(page_start, page, 2048)) {
        resp.resp.FlashPage.error_number = 5;
        break;
      }
      
      flash_lock();
      
      resp.resp.FlashPage.error_number = 0;
    } break;

    case CommandID::RunProgram: {
      // action happens later, after response is sent
    } break;

    default: {
      return; // send nothing back if command is invalid
    } break;

  }
  
  if(resp.id) {
    write_object(resp, *p_checksumadder);
  }
  
  switch(msg.command) {
    case CommandID::Reset: {
      // make sure write finishes
      usart_send_blocking(USART2, 0);
      usart_wait_send_ready(USART2);
      
      scb_reset_system();
    } break;

    case CommandID::RunProgram: {
      // make sure write finishes
      usart_send_blocking(USART2, 0);
      usart_wait_send_ready(USART2);
      
      asm("mov SP, %0;bx %1;"::"r"(lastpage->program_stack_pointer), "r"(lastpage->program_reset_vector));
    } break;
    
    default: {
    } break;
  }
}



void usart_setup(void) {
  /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPAEN);

  /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

  /* Setup UART parameters. */
  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART2);
}

void write_byte(uint8_t byte) {
  usart_send_blocking(USART2, byte);
}

uint8_t read_byte() {
  usart_wait_recv_ready(USART2);
  return usart_recv(USART2);
}

int main() {
  rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPEEN);
  usart_setup();
  
  arm_bootloader::Packetizer<void (*)(uint8_t byte)>
    packetizer(write_byte);
  arm_bootloader::ChecksumAdder<arm_bootloader::Packetizer<void (*)(uint8_t byte)> >
    checksumadder(packetizer);
  p_checksumadder = &checksumadder;
  
  arm_bootloader::ObjectReceiver<Command, void(const Command &)>
    objectreceiver(messageReceived);
  arm_bootloader::ChecksumChecker<arm_bootloader::ObjectReceiver<Command, void(const Command &)> >
    cc(objectreceiver);
  arm_bootloader::Depacketizer<arm_bootloader::ChecksumChecker<arm_bootloader::ObjectReceiver<Command, void(const Command &)> > >
    depacketizer(cc);
  
  while(true) {
    depacketizer.handleRawByte(read_byte());
  }
}



extern "C" {

void _exit(int) {
  while(true);
}

void _kill(int) {
  ;
}

int _getpid() {
  return 0;
}

}
