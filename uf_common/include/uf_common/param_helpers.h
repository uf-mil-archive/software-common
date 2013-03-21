#ifndef UF_COMMON__PARAM_HELPERS_H
#define UF_COMMON__PARAM_HELPERS_H

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include "msg_helpers.h"


namespace uf_common {


inline Eigen::VectorXd get_Vector(ros::NodeHandle& nh, const std::string& name) {
    XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    Eigen::VectorXd res(my_list.size());
    for (int32_t i = 0; i < my_list.size(); i++) {
        if(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            res[i] = static_cast<double>(my_list[i]);
        else if(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
            res[i] = static_cast<int>(my_list[i]);
        else
            ROS_ASSERT_MSG(false, "param type = %i", my_list[i].getType());
    }
    return res;
}

inline Eigen::Vector3d get_Vector3(ros::NodeHandle& nh, const std::string& name) {
    return get_Vector(nh, name);
}

inline tf::Vector3 get_tfVector3(ros::NodeHandle& nh, const std::string& name) {
    return vec2vec(get_Vector3(nh, name));
}

inline tf::Quaternion get_tfQuaternion(ros::NodeHandle& nh, const std::string& name) {
    return vec2quat(get_Vector(nh, name));
}

}

#endif
