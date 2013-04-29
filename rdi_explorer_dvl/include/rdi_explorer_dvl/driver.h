#ifndef DRIVER_H
#define DRIVER_H

#include <cmath>
#include <fstream>

#include <boost/asio/serial_port.hpp>
#include <boost/foreach.hpp>
#include <boost/none.hpp>

#include <geometry_msgs/Vector3Stamped.h>
#include <uf_common/Float64Stamped.h>


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
            
            void read(boost::optional<geometry_msgs::Vector3Stamped> &res,
                      boost::optional<uf_common::Float64Stamped> &height_res) {
                res = boost::none;
                height_res = boost::none;
                
                if(!p.is_open()) {
                    open();
                    return;
                }
                
                ByteVec ensemble; ensemble.resize(4);
                
                if(!read_byte(ensemble[0])) return; // Header ID
                if(ensemble[0] != 0x7F) return;
                
                ros::Time stamp = ros::Time::now();
                
                if(!read_byte(ensemble[1])) return; // Data Source ID
                if(ensemble[1] != 0x7F) return;
                
                if(!read_byte(ensemble[2])) return; // Size low
                if(!read_byte(ensemble[3])) return; // Size high
                uint16_t ensemble_size = getu16le(ensemble.data() + 2);
                ensemble.resize(ensemble_size);
                for(int i = 4; i < ensemble_size; i++) {
                    if(!read_byte(ensemble[i])) return;
                }
                
                uint16_t checksum = 0; BOOST_FOREACH(uint16_t b, ensemble) checksum += b;
                uint16_t received_checksum; if(!read_short(received_checksum)) return;
                if(received_checksum != checksum) {
                    ROS_ERROR("Invalid DVL ensemble checksum. received: %i calculated: %i size: %i", received_checksum, checksum, ensemble_size);
                    return;
                }
                
                if(ensemble.size() < 6) return;
                for(int dt = 0; dt < ensemble[5]; dt++) {
                    int offset = getu16le(ensemble.data() + 6 + 2*dt);
                    if(ensemble.size() - offset < 2) continue;
                    uint16_t section_id = getu16le(ensemble.data() + offset);
                    
                    if(section_id == 0x5803) { // Bottom Track High Resolution Velocity
                        if(ensemble.size() - offset < 2 + 4*4) continue;
                        if(gets32le(ensemble.data() + offset + 2) == -3276801) { // -3276801 indicates no data
                            ROS_ERROR("DVL didn't return bottom velocity");
                            continue;
                        }
                        res = boost::make_optional(geometry_msgs::Vector3Stamped());
                        res->header.stamp = stamp;
                        res->vector.x = gets32le(ensemble.data() + offset + 2) * .01e-3;
                        res->vector.y = gets32le(ensemble.data() + offset + 6) * .01e-3;
                        res->vector.z = gets32le(ensemble.data() + offset + 10) * .01e-3;
                    } else if(section_id == 0x5804) { // Bottom Track Range
                        if(ensemble.size() - offset < 2 + 4*3) continue;
                        if(gets32le(ensemble.data() + offset + 10) <= 0) {
                            ROS_ERROR("DVL didn't return height over bottom");
                            continue;
                        }
                        height_res = boost::make_optional(uf_common::Float64Stamped());
                        height_res->header.stamp = stamp;
                        height_res->data = gets32le(ensemble.data() + offset + 10) * 0.1e-3;
                    }
                }
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
