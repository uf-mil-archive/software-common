#include "ueye2/UEyeCamera.h"
#include "ueye2/UEyeConfig.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include <boost/scoped_ptr.hpp>

struct Node {
    boost::scoped_ptr<UEyeCamera> cam;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    dynamic_reconfigure::Server<ueye2::UEyeConfig> dyn_srv;
    
    Node(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) :
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
        dyn_srv.setCallback(boost::bind(&Node::reconfigure_callback, this, _1, _2));
    }

    void reconfigure_callback(ueye2::UEyeConfig &config, uint32_t level) {
        cam->setGains(config.red_gain, config.green_gain, config.blue_gain);
        cam->setAutoFunction(UEyeCamera::AUTO_WHITEBALANCE, true);
        cam->setAutoBrightReference(config.auto_reference);
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
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    try {
        Node node(nh, private_nh);
        node.run();
    } catch (const std::exception &ex) {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        ROS_ERROR_STREAM("Unhandled exception: " << ex.what());
        return 1;
    }
}
