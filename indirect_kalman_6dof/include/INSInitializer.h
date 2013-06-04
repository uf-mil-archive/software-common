#ifndef INSINITIALIZER_H
#define INSINITIALIZER_H

#include "INS.h"

#include <Eigen/Dense>
#include <deque>
#include <boost/array.hpp>

class INSInitializer {
public:
    struct Config {
        unsigned int accel_samples;
        unsigned int mag_samples;
        unsigned int dvl_samples;
        unsigned int depth_samples;

        Eigen::Vector3d g_nav;
        Eigen::Vector3d m_nav;
        Eigen::Vector3d r_imu2depth;
        Eigen::Matrix<double, 4, 3> beam_mat;
    };

    INSInitializer(const Config &config);

    bool ready() const;
    INS initializeINS() const;
    void clear();

    void updateAccel(const Eigen::Vector3d &y_a);
    void updateMag(const Eigen::Vector3d &y_m);
    void updateDVL(const Eigen::Vector4d &y_d);
    void updateDepth(double y_z);

private:
    Config config;

    std::deque<Eigen::Vector3d> y_a_log;
    std::deque<Eigen::Vector3d> y_m_log;
    std::deque<Eigen::Vector4d> y_d_log;
    std::deque<double> y_z_log;
};

#endif
