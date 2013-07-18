#ifndef UEYENODE_H
#define UEYENODE_H

#include "ueye2/UEyeCamera.h"
#include "ueye2/UEyeConfig.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <boost/scoped_ptr.hpp>

class UEyeNode {
public:
    UEyeNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);

    void runOnce();
    
private:
    void reconfigureCallback(ueye2::UEyeConfig &config, uint32_t level);
    bool setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request& req,
                               sensor_msgs::SetCameraInfo::Response& rsp);
    void applyConfig(const ueye2::UEyeConfig &config);

    std::string frame_id;
    std::string calibration_file;
    boost::scoped_ptr<UEyeCamera> cam;
    sensor_msgs::CameraInfo camera_info;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    ros::Publisher info_pub;
    dynamic_reconfigure::Server<ueye2::UEyeConfig> dyn_srv;
    boost::optional<ueye2::UEyeConfig> next_config;
    ros::ServiceServer info_srv;
};

#endif
