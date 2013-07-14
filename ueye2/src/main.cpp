#include "ueye2/UEyeCamera.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/scoped_ptr.hpp>

struct Node {
    boost::scoped_ptr<UEyeCamera> cam;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    
    Node() :
        private_nh("~"),
        it(nh)
    {
        int hbins;
        if (!private_nh.getParam("hbins", hbins)) {
            hbins = 1;
        }

        int vbins;
        if (!private_nh.getParam("vbins", vbins)) {
            vbins = 1;
        }

        int buffers;
        if (!private_nh.getParam("buffers", buffers)) {
            buffers = 2;
        }

        cam.reset(new UEyeCamera(0, hbins, vbins, buffers));
        cam->setAutoFunction(UEyeCamera::AUTO_WHITEBALANCE, true);
        cam->setAutoFunction(UEyeCamera::AUTO_SHUTTER, true);
        cam->setAutoFunction(UEyeCamera::AUTO_FRAMERATE, true);
        cam->setAutoBrightReference(0.5);
        image_pub = it.advertise("image_raw", 1);
    }

    void run() {
        ROS_INFO("UEye camera running");

        ros::Time last_frame = ros::Time::now();
        bool last_frame_warning = false;
        ros::Rate r(10);
        while (true) {
            ros::spinOnce();
            if (!ros::ok())
                break;
            
            if (image_pub.getNumSubscribers() == 0) {
                cam->stop();
                r.sleep();
                continue;
            }

            cam->start();
            if (!publish_image()) {
                if (ros::Time::now() - last_frame > ros::Duration(1)) {
                    if (!last_frame_warning) {
                        ROS_WARN("No frame from camera for over 1 second");
                        last_frame_warning = true;
                    }
                }
                continue;
            }
                
            last_frame = ros::Time::now();
            if (last_frame_warning) {
                ROS_INFO("Camera frames returned");
                last_frame_warning = false;
            }
        }
    }

    bool publish_image() {
        sensor_msgs::Image image;
        image.width = cam->getWidth();
        image.height = cam->getHeight();
        image.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
        image.data.resize(image.width*image.height*4);
        image.step = image.width*4;
        
        if (!cam->getBayeredImage(&image.data.front(), image.data.size(), 100))
            return false;
        image.header.stamp = ros::Time::now();
        image_pub.publish(image);
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ueye2");
    try {
        Node node;
        node.run();
    } catch (const std::exception &ex) {
        ROS_ERROR_STREAM("Unhandled exception: " << ex.what());
        return 1;
    }
}
