#include <Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>

bool intersect_plane_sphere(Eigen::Vector3d plane_axis1, Eigen::Vector3d plane_axis2,
        Eigen::Vector3d sphere_pos, double sphere_radius,
        double &hit1_axis1, double &hit1_axis2,
        double &hit2_axis1, double &hit2_axis2);

void sphere_query(const std::vector<Eigen::Vector3d>& sumimage, const sensor_msgs::CameraInfoConstPtr& cam_info, Eigen::Vector3d sphere_pos_camera, double sphere_radius, Eigen::Vector3d &total_color, double &count);
