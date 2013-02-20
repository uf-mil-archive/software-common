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
            
            const std::string port;
            const int baudrate;
            boost::asio::io_service io;
            boost::asio::serial_port p;
            
            bool read_byte(uint8_t &res) {
                while(true) {
                    try {
                        p.read_some(boost::asio::buffer(&res, sizeof(res)));
                        return true;
                    } catch(const std::exception &exc) {
                        ROS_ERROR("error on read: %s; reopening", exc.what());
                        open();
                        return false;
                    }
                }
            }
            bool read_short(uint16_t &x) {
                uint8_t low; if(!read_byte(low)) return false;
                uint8_t high; if(!read_byte(high)) return false;
                x = 256 * high + low;
                return true;
            }
            
            void open() {
                try {
                    p.close();
                    p.open(port);
                    p.set_option(boost::asio::serial_port::baud_rate(baudrate));
                    return;
                } catch(const std::exception &exc) {
                    ROS_ERROR("error on open(%s): %s; reopening after delay", port.c_str(), exc.what());
                    boost::this_thread::sleep(boost::posix_time::seconds(1));
                }
            }
            
        public:
            Device(const std::string port, int baudrate) : port(port), baudrate(baudrate), p(io) {
                // open is called on first read() in the _polling_ thread
            }
            
            bool read(geometry_msgs::Vector3Stamped &res) {
                if(!p.is_open()) {
                    open();
                    return false;
                }
                
                ByteVec ensemble; ensemble.resize(4);
                
                if(!read_byte(ensemble[0])) return false; // Header ID
                if(ensemble[0] != 0x7F) return false;
                
                ros::Time stamp = ros::Time::now();
                
                if(!read_byte(ensemble[1])) return false; // Data Source ID
                if(ensemble[1] != 0x7F) return false;
                
                if(!read_byte(ensemble[2])) return false; // Size low
                if(!read_byte(ensemble[3])) return false; // Size high
                uint16_t ensemble_size = getu16le(ensemble.data() + 2);
                ensemble.resize(ensemble_size);
                for(int i = 4; i < ensemble_size; i++) {
                    if(!read_byte(ensemble[i])) return false;
                }
                
                uint16_t checksum = 0; BOOST_FOREACH(uint16_t b, ensemble) checksum += b;
                uint16_t received_checksum; if(!read_short(received_checksum)) return false;
                if(received_checksum != checksum) {
                    ROS_ERROR("Invalid DVL ensemble checksum. received: %i calculated: %i size: %i", received_checksum, checksum, ensemble_size);
                    return false;
                }
                
                for(int dt = 0; dt < ensemble[5]; dt++) {
                    int offset = getu16le(ensemble.data() + 6 + 2*dt);
                    if(ensemble.size() - offset < 2+4*4) continue;
                    if(getu16le(ensemble.data() + offset) != 0x5803) continue;
                    res.header.stamp = stamp;
                    if(gets32le(ensemble.data() + offset + 2) == -3276801) { // -3276801 indicates no data
                        ROS_ERROR("DVL didn't return bottom velocity");
                        return false;
                    }
                    res.vector.x = gets32le(ensemble.data() + offset + 2) * .01e-3;
                    res.vector.y = gets32le(ensemble.data() + offset + 6) * .01e-3;
                    res.vector.z = gets32le(ensemble.data() + offset + 10) * .01e-3;
                    return true;
                }
                
                return false;
            }
            
            
            void send_heartbeat() {
                double maxdepth = 15;
                
                std::stringstream buf;
                buf << "CR0\r"; // load factory settings (won't change baud rate)
                buf << "#BJ 100 110 000\r"; // enable only bottom track high res velocity and bottom track range
                //buf << "#BK2\r"; // send water mass pings when bottom track pings fail
                //buf << "#BL7,36,46\r"; // configure near layer and far layer to 12 and 15 feet
                buf << "ES0\r"; // 0 salinity
                buf << "EX10010\r"; // transform results to ship XYZ, allow 3 beam solutions
                buf << "EZ10000010\r"; // configure sensor sources. Provide manual data for everything except speed of sound and temperature
                buf << "BX" << std::setw(5) << std::setfill('0') << (int)(maxdepth * 10 + 0.5) << '\r'; // configure max depth
                buf << "TT2012/03/04, 05:06:07\r"; // set RTC
                buf << "CS\r"; // start pinging
                
                std::string str = buf.str();
                
                try {
                    size_t written = 0;
                    while(written < str.size()) {
                        written += p.write_some(boost::asio::buffer(str.data() + written, str.size() - written));
                    }
                } catch(const std::exception &exc) {
                    ROS_ERROR("error on write: %s; dropping", exc.what());
                }
            }
            
            void abort() {
                p.close();
            }
    };
    
}

#endif
