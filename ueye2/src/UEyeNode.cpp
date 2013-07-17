#include "ueye2/UEyeNode.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse_ini.h>

#include <boost/shared_ptr.hpp>
#include <fstream>

UEyeNode::UEyeNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) :
    nh(nh),
    private_nh(private_nh),
    it(nh)
{
    int id = 0;
    private_nh.getParam("id", id);

    int hbins = 1;
    private_nh.getParam("hbins", hbins);

    int vbins = 1;
    private_nh.getParam("vbins", vbins);

    int buffers = 2;
    private_nh.getParam("buffers", buffers);

    bool autowhitebalance = true;
    private_nh.getParam("autowhitebalance", autowhitebalance);
        
    double fps = 0;
    private_nh.getParam("framerate", fps);

    private_nh.getParam("frame_id", frame_id);
    
    private_nh.getParam("calibration_file", calibration_file);
    std::string camera_name;
    if (!camera_calibration_parsers::readCalibrationIni(calibration_file, camera_name, camera_info)) {
        ROS_WARN("Failed to load intrinsics for camera from file");
    }
    
    cam.reset(new UEyeCamera(id, hbins, vbins, buffers));
    cam->setAutoFunction(UEyeCamera::AUTO_WHITEBALANCE, autowhitebalance);
    cam->setAutoFunction(UEyeCamera::AUTO_SHUTTER, true);
    cam->setAutoFunction(UEyeCamera::AUTO_GAIN, false);
    if (fps > 0) {
        cam->setFrameRate(fps);
    } else {
        cam->setAutoFunction(UEyeCamera::AUTO_FRAMERATE, true);
    }        

    image_pub = it.advertise("image_raw", 1);
    info_pub = this->nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    dyn_srv.setCallback(boost::bind(&UEyeNode::reconfigureCallback, this, _1, _2));
    info_srv = this->nh.advertiseService("set_camera_info",
                                         &UEyeNode::setCameraInfoCallback, this);
}

void UEyeNode::reconfigureCallback(ueye2::UEyeConfig &config, uint32_t level) {
    cam->setGains(config.red_gain, config.green_gain, config.blue_gain);
    cam->setAutoBrightReference(config.auto_reference);
}

bool UEyeNode::setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request& req,
                                     sensor_msgs::SetCameraInfo::Response& rsp) {
    ROS_INFO("New camera info received");
    camera_info = req.camera_info;
    std::ofstream out(calibration_file.c_str());
    rsp.success = camera_calibration_parsers::writeCalibrationIni(out, "ueye", req.camera_info);
    return true;
}
    
void UEyeNode::runOnce() {
    if (image_pub.getNumSubscribers() == 0) {
        cam->stop();
        ros::Duration(.1).sleep();
        return;
    }
    cam->start();

    boost::shared_ptr<sensor_msgs::Image> image
        = boost::make_shared<sensor_msgs::Image>();
    image->width = cam->getWidth();
    image->height = cam->getHeight();
    image->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    image->data.resize(image->width*image->height*4);
    image->step = image->width*4;
    image->header.frame_id = frame_id;
    if (!cam->getBayeredImage(&image->data.front(), image->data.size(), 100)) {
        return;
    }
    image->header.stamp = ros::Time::now();
    image_pub.publish(image);
    camera_info.header = image->header;
    info_pub.publish(camera_info);
}
