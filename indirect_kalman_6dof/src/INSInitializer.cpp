#include "INSInitializer.h"
#include <numeric>

#include <ros/ros.h>

INSInitializer::INSInitializer(const Config &config) :
    config(config) { }

bool INSInitializer::ready() const {
    return y_a_log.size() >= config.accel_samples
        && y_m_log.size() >= config.mag_samples
        && y_d_log.size() >= config.dvl_samples
        && y_z_log.size() >= config.depth_samples;
}

namespace {
    template <typename Container>
    typename Container::value_type mean(const Container &cont,
                                        typename Container::value_type val) {
        for (typename Container::const_iterator i = cont.begin();
             i != cont.end();
             ++i) {
            val += *i;
        }

        return val / cont.size();
    }
}

INS INSInitializer::initializeINS() const {
    // Average up all accel and mag samples
    Eigen::Vector3d y_a = mean(y_a_log, Eigen::Vector3d::Zero());
    Eigen::Vector3d y_m = mean(y_m_log, Eigen::Vector3d::Zero());

    // Use TRIAD to compute a rotation matrix
    Eigen::Vector3d a = -config.g_nav.normalized();
    Eigen::Vector3d a_hat = y_a.normalized();
    Eigen::Vector3d m = a.cross(config.m_nav).normalized();
    Eigen::Vector3d m_hat = a_hat.cross(y_m).normalized();
    Eigen::Matrix3d R_imu2nav =
        (Eigen::Matrix3d() << a, m, a.cross(m)).finished() *
        (Eigen::Matrix3d() << a_hat, m_hat, a_hat.cross(m_hat)).finished().transpose();

    // Sum up all the DVL readings
    Eigen::Vector4d y_d = mean(y_d_log, Eigen::Vector4d::Zero());

    // Use least squares with the beam matrix to determine our velocity
    Eigen::Vector3d v_imu_init =
        (config.beam_mat.transpose()*config.beam_mat).lu().solve( // TODO fix guide
            config.beam_mat.transpose() * y_d);

    // Use the depth sensor to determine our z position
    double p_nav_z = -mean(y_z_log, 0) - (R_imu2nav*config.r_imu2depth)[2];

    // Initialize an INS
    return INS(Eigen::Quaterniond(R_imu2nav),
               R_imu2nav * v_imu_init,
               Eigen::Vector3d(0, 0, p_nav_z),
               config.g_nav);
}

void INSInitializer::clear() {
    y_a_log.clear();
    y_m_log.clear();
    y_d_log.clear();
    y_z_log.clear();
}

void INSInitializer::updateAccel(const Eigen::Vector3d &y_a) {
    y_a_log.push_back(y_a);
    if (y_a_log.size() > config.accel_samples)
        y_a_log.pop_front();
}

void INSInitializer::updateMag(const Eigen::Vector3d &y_m) {
    y_m_log.push_back(y_m);
    if (y_m_log.size() > config.mag_samples)
        y_m_log.pop_front();
}

void INSInitializer::updateDVL(const Eigen::Vector4d &y_d) {
    y_d_log.push_back(y_d);
    if (y_d_log.size() > config.dvl_samples)
        y_d_log.pop_front();
}

void INSInitializer::updateDepth(double y_z) {
    y_z_log.push_back(y_z);
    if (y_z_log.size() > config.depth_samples)
        y_z_log.pop_front();
}
