#ifndef DRIVER_H
#define DRIVER_H

#include <cmath>
#include <fstream>

#include <boost/asio/serial_port.hpp>
#include <boost/foreach.hpp>

#include <geometry_msgs/Vector3Stamped.h>


namespace rdi_explorer_dvl {
    static uint16_t getu16le(uint8_t* i) { return *i | (*(i+1) << 8); }
    static int32_t gets32le(uint8_t* i) { return *i | (*(i+1) << 8) | (*(i+2) << 16) | (*(i+3) << 24); }
    
    class Device {
        private:
            typedef std::vector<boost::uint8_t> ByteVec;
            
            boost::asio::io_service io;
            boost::asio::serial_port p;
            
            void send_packet(ByteVec out) {
                p.write_some(boost::asio::buffer(out.data(), out.size()));
            }
            
            uint8_t read_byte() {
                uint8_t res; p.read_some(boost::asio::buffer(&res, sizeof(res)));
                return res;
            }
            uint16_t read_short() {
                uint8_t low = read_byte();
                return 256 * read_byte() + low;
            }
            
        public:
            Device(const std::string port, int baudrate) : p(io) {
                p.open(port);
                p.set_option(boost::asio::serial_port::baud_rate(baudrate));
            }
            
            geometry_msgs::Vector3Stamped read() {
            start:
                ByteVec ensemble;
                ensemble.push_back(read_byte()); if(ensemble[0] != 0x7F) goto start; // Header ID
                ros::Time stamp = ros::Time::now();
                ensemble.push_back(read_byte()); if(ensemble[1] != 0x7F) goto start; // Data Source ID
                ensemble.push_back(read_byte());
                ensemble.push_back(read_byte());
                uint16_t ensemble_size = ensemble[2] + 256 * ensemble[3];
                ensemble.resize(ensemble_size);
                p.read_some(boost::asio::buffer(ensemble.data() + 4, ensemble_size - 4));
                
                uint16_t checksum = 0; BOOST_FOREACH(uint16_t b, ensemble) checksum += b;
                if(checksum != read_short()) {
                    ROS_ERROR("invalid dvl message checksum");
                    goto start;
                }
                
                //bla
                for(int dt = 0; dt < ensemble[5]; dt++) {
                    int offset = getu16le(ensemble.data() + 6 + 2*dt);
                    if(ensemble.size() - offset < 2+4*4) continue;
                    if(getu16le(ensemble.data() + offset) != 0x5803) continue;
                    geometry_msgs::Vector3Stamped res;
                    res.header.stamp = stamp;
                    res.vector.x = gets32le(ensemble.data() + offset + 2) * .01e-3;
                    res.vector.y = gets32le(ensemble.data() + offset + 6) * .01e-3;
                    res.vector.z = gets32le(ensemble.data() + offset + 10) * .01e-3;
                    return res;
                }
                
                goto start;
            }
            
            void send_heartbeat() {
                //send_packet(ByteVec()); // heartbeat
                //uint8_t msg[] = {4, 1, 20}; send_packet(ByteVec(msg, msg + sizeof(msg)/sizeof(msg[0]))); // StartPublishing 20hz
            }
    };
    
}

#endif
