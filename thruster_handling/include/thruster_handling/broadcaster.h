#ifndef THRUSTER_HANDLING_BROADCASTER_H
#define THRUSTER_HANDLING_BROADCASTER_H

#include <Eigen/Dense>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

#include "thruster_handling/ThrusterCommand.h"
#include "thruster_handling/ThrusterInfo.h"

namespace thruster_handling {


class ThrusterBroadcaster {
private:
  ros::NodeHandle &nh;
  std::string frame_id;
  std::string id;
  ros::Duration lifetime;
  Eigen::Vector3d position;
  Eigen::Vector3d direction;
  double min_force;
  double max_force;
  Eigen::Vector3d torque_per_force;
  boost::function<void(double)> command_callback;
  ros::Publisher _pub;
  ros::Subscriber _command_sub;
  
  void _command_callback(const ThrusterCommandConstPtr &msgp) {
    command_callback(msgp->force);
  }

public:
  ThrusterBroadcaster(ros::NodeHandle &nh,
                      std::string frame_id,
                      std::string id,
                      ros::Duration lifetime,
                      Eigen::Vector3d position,
                      Eigen::Vector3d direction,
                      double min_force,
                      double max_force,
                      Eigen::Vector3d torque_per_force,
                      boost::function<void(double)> command_callback) :
    nh(nh),
    frame_id(frame_id),
    id(id),
    lifetime(lifetime),
    position(position),
    direction(direction),
    min_force(min_force),
    max_force(max_force),
    torque_per_force(torque_per_force),
    command_callback(command_callback) {
    
    _pub = nh.advertise<ThrusterInfo>("thrusters/info", 10);
    _command_sub = nh.subscribe<ThrusterCommand>("thrusters/command/" + id,
      0, boost::bind(&ThrusterBroadcaster::_command_callback, this, _1));
  }
  
  void send(bool active=true) {
    ThrusterInfo ti;
    ti.header.stamp = ros::Time::now();
    ti.header.frame_id = frame_id;
    ti.id = id;
    ti.lifetime = lifetime;
    ti.active = active;
    tf::pointEigenToMsg(position, ti.position);
    tf::vectorEigenToMsg(direction, ti.direction);
    ti.min_force = min_force;
    ti.max_force = max_force;
    tf::vectorEigenToMsg(torque_per_force, ti.torque_per_force);
    _pub.publish(ti);
  }
};


}

#endif
