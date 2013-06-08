#ifndef NAVIGATIONCOMPUTER_H
#define NAVIGATIONCOMPUTER_H

#include "Kalman.h"
#include "INS.h"
#include "INSInitializer.h"

#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <vector>

class NavigationComputer {
public:
    struct Config {
        double T_imu;
        double T_kalman;
        double T_kalman_correction;

        Kalman::PredictConfig predict;
        Kalman::UpdateConfig update;
        INSInitializer::Config init;

        bool verify_timestamps;
    };

    NavigationComputer(const Config &config);

    struct Stats {
        unsigned int y_m_count;
        unsigned int y_d_count;
        unsigned int y_z_count;
    };

    struct State {
        INS::State filt;
        Kalman::StateMat cov;
        Stats stats;
        double ins_time;
    };

    boost::optional<State> getState() const;
    INS::Error getINSError() const;

    void updateINS(const INS::Measurement &measurement, double measurement_time);
    void updateMag(const Eigen::Vector3d &y_m, double measurement_time);
    void updateDVL(const Eigen::Matrix<double, 4, 1> &y_d,
                   const boost::array<bool, 4> &d_valid,
                   double measurement_time);
    void updateDepth(double y_z, double measurement_time);

    void run(double run_time);

private:
    // run sub-methods
    void tryInitINS(double run_time);
    void predict(int updates);
    void update();

    bool verifyKalmanTime(const std::string &sensor, double measurement_time);

    Config config;
    boost::optional<INS> ins;
    double ins_time;
    INSInitializer ins_init;

    Kalman kalman;
    double kalman_time;
    double last_correction_time;

    boost::optional<Eigen::Vector3d> y_m;
    Eigen::Vector4d y_d;
    boost::array<bool, 4> d_valid;
    boost::optional<double> y_z;

    Stats stats;
};

#endif
