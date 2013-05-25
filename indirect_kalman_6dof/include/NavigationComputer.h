#ifndef NAVIGATIONCOMPUTER_H
#define NAVIGATIONCOMPUTER_H

#include "Kalman.h"
#include "INS.h"
#include "INSInitializer.h"

#include <boost/array.hpp>
#include <boost/optional.hpp>

class NavigationComputer {
public:
    struct Config {
        double T_imu;
        double T_kalman;
        double T_kalman_correction;

        Kalman::PredictConfig predict;
        Kalman::UpdateConfig update;
        INSInitializer::Config init;
    };

    NavigationComputer(const Config &config);

    struct State {
        INS::State filt;
        Eigen::Matrix<double, 15, 15> cov;
    };

    boost::optional<State> getState() const;
    INS::Error getINSError() const;

    void updateINS(const INS::Measurement &measurement, double measurement_time);
    void updateMag(const Eigen::Vector3d &y_m);
    void updateDVL(const Eigen::Matrix<double, 4, 1> &y_d,
                   const boost::array<bool, 4> &d_valid);
    void updateDepth(double y_z);

    void run(double run_time);

private:
    Config config;
    boost::optional<INS> ins;
    double ins_time;
    INSInitializer ins_init;

    Kalman kalman;
    double kalman_time;
    double last_correction_time;

    boost::optional<Eigen::Vector3d> y_a;
    boost::optional<Eigen::Vector3d> y_m;
    Eigen::Vector4d y_d;
    boost::array<bool, 4> d_valid;
    boost::optional<double> y_z;
};

#endif
