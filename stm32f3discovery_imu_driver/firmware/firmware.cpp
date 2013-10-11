#include <cstring>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/stm32/f3/usart.h>
#include <libopencm3/stm32/f3/i2c.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>

#include "sha256.h"
#include <arm_bootloader/subbus_protocol.h>

#include "protocol.h"
#include "i2c.h"

using namespace stm32f3discovery_imu_driver;

arm_bootloader::ChecksumAdder<arm_bootloader::Packetizer<void (*)(uint8_t byte)> > *p_checksumadder;

void messageReceived(const Command &msg) {
  if(msg.dest != 0x1234) return;
  
  Response resp; memset(&resp, 0, sizeof(resp));
  resp.id = msg.id;
  
  switch(msg.command) {
  
    case CommandID::Reset: {
      // action happens later, after response is sent
    } break;
  
    case CommandID::GetStatus: {
      resp.resp.GetStatus.magic = GetStatusResponse::MAGIC_VALUE;
    } break;
    
    case CommandID::GetIMUData: {
      i2c_read_imu(resp.resp.GetIMUData);
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
	i2c_setup();
  
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
