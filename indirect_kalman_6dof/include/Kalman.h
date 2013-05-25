#ifndef KALMAN_H
#define KALMAN_H

#include "INS.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <fstream>

class Kalman {
public:
    Kalman();

    struct PredictConfig {
        Eigen::Matrix3d R_g;
        Eigen::Matrix3d R_a;
        Eigen::Matrix3d Q_b_g;
        Eigen::Matrix3d Q_b_a;
    };

    const Eigen::Matrix<double, 15, 1> &getX() const { return x; }
    const Eigen::Matrix<double, 15, 15> &getP() const { return P; }
    INS::Error getINSError() const;
    void resetINSError();
    void reset();

    void predict(const INS::State &ins, const PredictConfig &config, double dt);

    struct UpdateConfig {
        Eigen::Matrix<double, 4, 3> beam_mat;
        Eigen::Vector3d r_imu2dvl;
        Eigen::Quaterniond q_imu2dvl;
        Eigen::Vector3d r_imu2depth;
        Eigen::Vector3d m_nav;

        Eigen::Matrix3d R_g;
        Eigen::Matrix3d R_a;
        Eigen::Matrix3d R_m;
        Eigen::Matrix4d R_d;
        double R_z;
    };

    void update(const Eigen::Matrix<double, 11, 1> &z,
                bool a_valid,
                bool m_valid,
                const boost::array<bool, 4> &d_valid,
                bool z_valid,
                const INS::State &ins,
                const UpdateConfig &config);

//private:
    Eigen::Matrix<double, 15, 1> x;
    Eigen::Matrix<double, 15, 15> P;

    std::ofstream dbg;
};

#endif
