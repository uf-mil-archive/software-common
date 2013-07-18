#include "ueye2/UEyeNode.h"

#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ueye2");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    try {
        UEyeNode node(nh, private_nh);
        ROS_INFO("UEye camera running");
        while (true) {
            node.runOnce();
            ros::spinOnce();
        }
    } catch (const std::exception &ex) {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        ROS_ERROR_STREAM("Unhandled exception: " << ex.what());
        return 1;
    }
}
