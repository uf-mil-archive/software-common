#ifndef UF_COMMON__PARAM_HELPERS_H
#define UF_COMMON__PARAM_HELPERS_H

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include "msg_helpers.h"


namespace uf_common {

inline Eigen::MatrixXd get_Matrix(ros::NodeHandle& nh, const std::string& name) {
    XmlRpc::XmlRpcValue my_list; ROS_ASSERT_MSG(nh.getParam(name, my_list),
        "param %s is missing", name.c_str());
    ROS_ASSERT_MSG(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray,
        "param %s must be a list", name.c_str());

    ROS_ASSERT_MSG(my_list.size() >= 1,
        "param %s is zero-length", name.c_str());
    Eigen::MatrixXd res(my_list.size(), my_list[0].size());
    for (int32_t i = 0; i < my_list.size(); i++) {
        XmlRpc::XmlRpcValue row = my_list[i];
        ROS_ASSERT_MSG(row.getType() == XmlRpc::XmlRpcValue::TypeArray,
            "param %s[%i] must be a list", name.c_str(), i);
        ROS_ASSERT_MSG(row.size() == my_list[0].size(),
            "param %s[%i]'s length doesn't match", name.c_str(), i);
        for (int32_t j = 0; j < row.size(); j++) {
            XmlRpc::XmlRpcValue entry = row[j];
            if(entry.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                res(i, j) = static_cast<double>(entry);
            else if(entry.getType() == XmlRpc::XmlRpcValue::TypeInt)
                res(i, j) = static_cast<int>(entry);
            else {
                ROS_ASSERT_MSG(false, "param type = %i", entry.getType());
            }
        }
    }
    return res;
}

inline Eigen::VectorXd get_Vector(ros::NodeHandle& nh, const std::string& name) {
    XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    Eigen::VectorXd res(my_list.size());
    for (int32_t i = 0; i < my_list.size(); i++) {
        if(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            res[i] = static_cast<double>(my_list[i]);
        else if(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
            res[i] = static_cast<int>(my_list[i]);
        else {
            ROS_ASSERT_MSG(false, "param type = %i", my_list[i].getType());
        }
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


template<typename T, int N>
bool _getParam(ros::NodeHandle &nh, const std::string &name, Eigen::Matrix<T, N, 1> &res) {
  XmlRpc::XmlRpcValue my_list; if(!nh.getParam(name, my_list)) return false;
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  res.resize(my_list.size());
  for (int32_t i = 0; i < my_list.size(); i++) {
    if(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      res(i) = static_cast<double>(my_list[i]);
    } else if(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      res(i) = static_cast<int>(my_list[i]);
    } else {
      ROS_ASSERT_MSG(false, "param type = %i", my_list[i].getType());
    }
  }
  return true;
}
template<typename T>
bool _getParam(ros::NodeHandle &nh, const std::string &name, T &res) {
  return nh.getParam(name, res);
}
template<>
bool _getParam(ros::NodeHandle &nh, const std::string &name, ros::Duration &res) {
  double dur; if(!nh.getParam(name, dur)) return false;
  res = ros::Duration(dur); return true;
}

template<typename T>
T getParam(ros::NodeHandle &nh, std::string name) {
  T res;
  if(!_getParam(nh, name, res)) {
    throw std::runtime_error("need " + name + " param");
  }
  return res;
}
template<typename T>
T getParam(ros::NodeHandle &nh, std::string name, T default_value) {
  T res;
  if(!_getParam(nh, name, res)) {
    return default_value;
  }
  return res;
}


}

#endif
