#include "ueye2/UEyeNode.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/shared_ptr.hpp>

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
    dyn_srv.setCallback(boost::bind(&UEyeNode::reconfigureCallback, this, _1, _2));
}

void UEyeNode::reconfigureCallback(ueye2::UEyeConfig &config, uint32_t level) {
    cam->setGains(config.red_gain, config.green_gain, config.blue_gain);
    cam->setAutoFunction(UEyeCamera::AUTO_WHITEBALANCE, true);
    cam->setAutoBrightReference(config.auto_reference);
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
    if (!cam->getBayeredImage(&image->data.front(), image->data.size(), 100)) {
        return;
    }
    image->header.stamp = ros::Time::now();
    image_pub.publish(image);
}
