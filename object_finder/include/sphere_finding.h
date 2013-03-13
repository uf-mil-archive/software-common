#include <Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>

void sphere_query(const std::vector<Eigen::Vector3d>& sumimage, const sensor_msgs::CameraInfoConstPtr& cam_info, Eigen::Vector3d sphere_pos_camera, double sphere_radius, Eigen::Vector3d &total_color, double &count, std::vector<int>* dbg_image=NULL);
