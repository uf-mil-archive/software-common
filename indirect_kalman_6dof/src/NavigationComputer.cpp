#include "NavigationComputer.h"
#include <ros/console.h>
#include <algorithm>

NavigationComputer::NavigationComputer(const Config &config) :
    config(config),
    ins_time(0),
    ins_init(config.init),
    kalman_time(0),
    last_correction_time(0)
{
    y_d.fill(0);
    std::fill(d_valid.begin(), d_valid.end(), false);
}

boost::optional<NavigationComputer::State> NavigationComputer::getState() const {
    if (!ins) {
        return boost::none;
    }

    State state;
    state.filt = ins->getState();
    state.filt.correct(kalman.getINSError());
    state.cov = kalman.getP();
    return state;
}

void NavigationComputer::updateINS(const INS::Measurement &measurement,
                                   double measurement_time) {
    if (!ins) {
        ins_init.updateAccel(measurement.y_a);
        return;
    }

    double T_delta = measurement_time - ins_time;
    int updates = static_cast<int>(round(T_delta / config.T_imu));

    if (updates <= 0) {
        ROS_WARN_STREAM("Late IMU measurement, dropping "
                        "(T_delta " << T_delta << ")");
        return;
    }

    if (updates > 5) {
        ROS_WARN_STREAM("Dropped " << updates << " INS messages, skipping ahead");
        ins->update(measurement, config.T_imu);
        ins_time += updates*config.T_imu;
        return;
    }

    for (int i=0; i<updates; i++) {
        ins->update(measurement, config.T_imu);
        ins_time += config.T_imu;
    }

    y_a = measurement.y_a;
}

void NavigationComputer::updateMag(const Eigen::Vector3d &y_m) {
    if (!ins) {
        ins_init.updateMag(y_m);
        return;
    }

    this->y_m = y_m;
}

void NavigationComputer::updateDVL(const Eigen::Matrix<double, 4, 1> &y_d,
                                   const boost::array<bool, 4> &d_valid) {
    if (!ins) {
        if (std::count(d_valid.begin(), d_valid.end(), true) == 4) {
            ins_init.updateDVL(y_d);
        }
        return;
    }

    this->y_d = y_d;
    this->d_valid = d_valid;

    ROS_INFO_STREAM("DVL! " << d_valid[0] << " " << d_valid[1] << " "
                    << d_valid[2] << " " << d_valid[3]);
}

void NavigationComputer::updateDepth(double y_z) {
    if (!ins) {
        ins_init.updateDepth(y_z);
    }

    this->y_z = y_z;
}

void NavigationComputer::run(double run_time) {
    // TODO split up
    // TODO don't allow kalman to get ahead of INS
    if (!ins) {
        if (!ins_init.ready()) {
            return;
        }

        ins = ins_init.initializeINS();

        const INS::State &state = ins->getState();
        ROS_INFO_STREAM("Initialized INS");
        ROS_INFO_STREAM("q_imu2nav "
                        << state.q_imu2nav.w() << " "
                        << state.q_imu2nav.x() << " "
                        << state.q_imu2nav.y() << " "
                        << state.q_imu2nav.z());
        ROS_INFO_STREAM("v_nav "
                        << state.v_nav[0] << " "
                        << state.v_nav[1] << " "
                        << state.v_nav[2]);
        ROS_INFO_STREAM("p_nav "
                        << state.p_nav[0] << " "
                        << state.p_nav[1] << " "
                        << state.p_nav[2]);

        ins_init.clear();
        kalman.reset();
        ins_time = run_time;
        kalman_time = run_time;
        last_correction_time = run_time;
        return;
    }

    double T_delta = run_time - kalman_time;
    int updates = static_cast<int>(round(T_delta / config.T_kalman));

    for (int i = 0; i < updates; i++) {
        kalman.predict(ins->getState(), config.predict, config.T_kalman);
        kalman_time += config.T_kalman;
    }

    Eigen::Matrix3d R_imu2nav = ins->getState().q_imu2nav.matrix();

    Eigen::Matrix<double, 11, 1> z;
    z.fill(0);
    bool a_valid = false;
    bool m_valid = false;
    bool z_valid = false;
    if (y_a) {
        if (abs(y_a->norm() - ins->getState().g_nav.norm()) < 1e-2) {
            z.segment<3>(0) = *y_a - -R_imu2nav.transpose()*ins->getState().g_nav;
            a_valid = true;
        }
    }
    if (y_m) {
        z.segment<3>(3) = *y_m - R_imu2nav.transpose()*config.update.m_nav;
        m_valid = true;
    }
    z.segment<4>(6) = y_d - config.update.beam_mat*(
        R_imu2nav.transpose()*ins->getState().v_nav +
        ins->getState().w_imu.cross(config.update.r_imu2dvl));
    if (y_z) {
        z[10] = *y_z - -(ins->getState().p_nav + R_imu2nav*config.update.r_imu2depth)[2];
        z_valid = true;
    }

    if (!y_a && !y_m && std::count(d_valid.begin(), d_valid.end(), true) == 0 && !y_z) {
        return;
    }

    kalman.update(z, a_valid, m_valid, d_valid, z_valid, ins->getState(), config.update);

    y_a = boost::none;
    y_m = boost::none;
    std::fill(d_valid.begin(), d_valid.end(), false);
    y_z = boost::none;

    if (kalman_time - last_correction_time >= config.T_kalman_correction) {
        last_correction_time = kalman_time;
        ins->correct(kalman.getINSError());
        kalman.resetINSError();
    }
}
