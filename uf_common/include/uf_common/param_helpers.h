#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Dense>

namespace uf_common {


inline Eigen::Vector3d get_Vector3(ros::NodeHandle& nh, const std::string& name) {
    XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(my_list.size() == 3);

    Eigen::Vector3d res;
    for (uint32_t i = 0; i < 3; i++) {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        res[i] = static_cast<double>(my_list[i]);
    }
    return res;
}

inline tf::Vector3 get_tfVector3(ros::NodeHandle& nh, const std::string& name) {
    XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(my_list.size() == 3);
    
    tf::Vector3 res;
    for (uint32_t i = 0; i < 3; i++) {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        res[i] = static_cast<double>(my_list[i]);
    }
    return res;
}

inline tf::Quaternion get_tfQuaternion(ros::NodeHandle& nh, const std::string& name) {
    XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(my_list.size() == 4);
    
    tf::Quaternion res;
    for (uint32_t i = 0; i < 4; i++) {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        res[i] = static_cast<double>(my_list[i]);
    }
    return res;
}

}
