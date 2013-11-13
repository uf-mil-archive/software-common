#include <iostream>

#include <arm_bootloader/arm_bootloader.h>

#include <stm32f3discovery_imu_driver/protocol.h>

extern unsigned char firmware_bin[];
extern int firmware_bin_len;

using namespace stm32f3discovery_imu_driver;

int main(int argc, char **argv) {
  boost::asio::io_service io;

  boost::asio::serial_port sp(io);
  sp.open("/dev/ttyUSB0");
  sp.set_option(boost::asio::serial_port::baud_rate(115200));

  if(argc <= 1) {
    if(!arm_bootloader::attempt_bootload(sp, firmware_bin, firmware_bin_len)) {
      std::cout << "bootloading failed" << std::endl;
      return EXIT_FAILURE;
    }
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 65535);
  
  arm_bootloader::Reader<Response> reader(sp, 1000);
  
  arm_bootloader::SerialPortSink sps(sp);
  uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink>
    packetizer(sps);
  uf_subbus_protocol::ChecksumAdder<uf_subbus_protocol::Packetizer<arm_bootloader::SerialPortSink> >
    checksumadder(packetizer);
  
  while(true) {
    Command cmd; memset(&cmd, 0, sizeof(cmd));
    cmd.dest = 0x1234;
    cmd.id = dis(gen);
    cmd.command = CommandID::GetIMUData;
    write_object(cmd, checksumadder);
    
    boost::optional<Response> resp = reader.read(cmd.id);
    if(!resp) {
      std::cout << "timeout receiving imu packet!" << std::endl;
      continue;
    }
    std::cout << resp->resp.GetIMUData.linear_acceleration[0] << " ";
    std::cout << resp->resp.GetIMUData.linear_acceleration[1] << " ";
    std::cout << resp->resp.GetIMUData.linear_acceleration[2] << std::endl;
  }

  return EXIT_SUCCESS;
}
