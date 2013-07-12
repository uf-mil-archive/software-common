#include "ueye2/UEyeCamera.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/scoped_ptr.hpp>
#include <iostream>

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
        image_pub = it.advertise("image_raw", 1);
    }

    void run() {
        ros::Rate r(10);
        while (ros::ok()) {
            ros::spinOnce();
            if (image_pub.getNumSubscribers() == 0) {
                cam->stop();
                r.sleep();
                continue;
            }

            cam->start();
            publish_image();
        }
    }

    void publish_image() {
        sensor_msgs::Image image;
        image.width = cam->getWidth();
        image.height = cam->getHeight();
        image.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
        image.data.resize(image.width*image.height*4);
        image.step = image.width*4;
        
        if (!cam->getBayeredImage(&image.data.front(), image.data.size()))
            return;
        image.header.stamp = ros::Time::now();
        image_pub.publish(image);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ueye2");
    try {
        Node node;
        node.run();
    } catch (UEyeException &ex) {
        std::cout << ex.what() << std::endl;
        return 1;
    }
}
