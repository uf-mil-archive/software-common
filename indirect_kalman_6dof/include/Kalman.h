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
    };

    typedef Eigen::Matrix<double, 12, 1> StateVec;
    typedef Eigen::Matrix<double, 12, 12> StateMat;

    const StateVec &getX() const { return x; }
    const StateMat &getP() const { return P; }
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
        Eigen::Matrix3d R_m;
        Eigen::Matrix4d R_d;
        double R_z;
    };

    typedef Eigen::Matrix<double, 8, 1> MeasurementVec;

    void update(const MeasurementVec &z,
                bool m_valid,
                const boost::array<bool, 4> &d_valid,
                bool z_valid,
                const INS::State &ins,
                const UpdateConfig &config);

private:
    StateVec x;
    StateMat P;
};

#endif
