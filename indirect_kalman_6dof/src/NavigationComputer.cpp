#include "NavigationComputer.h"
#include "util.h"
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
    y_a_log.reserve(config.y_a_log_size);

    stats.y_a_count = 0;
    stats.y_m_count = 0;
    stats.y_d_count = 0;
    stats.y_z_count = 0;
}

boost::optional<NavigationComputer::State> NavigationComputer::getState() const {
    if (!ins) {
        return boost::none;
    }

    State state;
    state.filt = ins->getState();
    state.filt.correct(kalman.getINSError());
    state.cov = kalman.getP();
    state.stats = stats;
    return state;
}

void NavigationComputer::updateINS(const INS::Measurement &measurement,
                                   double measurement_time) {
    if (!ins) {
        ins_init.updateAccel(measurement.y_a);
        return;
    }

    // Calculate how many updates are necessary to bring INS
    // to this measurement.
    double T_delta = measurement_time - ins_time;
    int updates = static_cast<int>(round(T_delta / config.T_imu));

    if (updates < 0) {
        ROS_WARN_STREAM("Out of order IMU measurement, dropping "
                        "(T_delta " << T_delta << ")");
        return;
    }

    if (updates >= 20) {
        // too many dropped messages, just advance time
        ROS_WARN_STREAM("Dropped " << updates << " INS messages, skipping ahead");
        ins_time += updates*config.T_imu;
        return;
    }

    if (updates >= 3) {
        ROS_WARN_STREAM("Dropped " << updates << " INS messages");
    }

    // Run the correct number of INS updates
    for (int i=0; i<updates; i++) {
        ins->update(measurement, config.T_imu);
        ins_time += config.T_imu;
    }

    // Save the accelerometer reading for the accelerometer hack
    if (y_a_log.size() == config.y_a_log_size) {
        y_a_log.erase(y_a_log.begin());
    }
    y_a_log.push_back(measurement.y_a);
}

void NavigationComputer::updateMag(const Eigen::Vector3d &y_m,
                                   double measurement_time) {
    // if we don't have an INS, forward to the INSInitializer
    if (!ins) {
        ins_init.updateMag(y_m);
        return;
    }

    // Make sure this reading's timestamp checks out
    if (!verifyKalmanTime("mag", measurement_time)) {
        return;
    }

    // Save it for the update
    this->y_m = y_m;
}

void NavigationComputer::updateDVL(const Eigen::Matrix<double, 4, 1> &y_d,
                                   const boost::array<bool, 4> &d_valid,
                                   double measurement_time) {
    if (!ins) {
        // only give to INSInitializer if all 4 beams are valid
        if (std::count(d_valid.begin(), d_valid.end(), true) == 4) {
            ins_init.updateDVL(y_d);
        }
        return;
    }

    if (!verifyKalmanTime("DVL", measurement_time)) {
        return;
    }

    this->y_d = y_d;
    this->d_valid = d_valid;
}

void NavigationComputer::updateDepth(double y_z, double measurement_time) {
    if (!ins) {
        ins_init.updateDepth(y_z);
        return;
    }

    if (!verifyKalmanTime("depth", measurement_time)) {
        return;
    }

    this->y_z = y_z;
}

void NavigationComputer::run(double run_time) {
    // if we don't have an INS yet, try to initialize one
    if (!ins) {
        tryInitINS(run_time);
        return;
    }

    // compute how many predictions are necessary to bring kalman
    // even with the INS
    double T_delta = ins_time - kalman_time;
    int predicts = static_cast<int>(round(T_delta / config.T_kalman));

    // run the correct number of predicts
    predict(predicts);

    // run a single update
    update();

    // reset the filter if sufficient time has passed
    if (kalman_time - last_correction_time >= config.T_kalman_correction) {
        last_correction_time = kalman_time;
        ins->correct(kalman.getINSError());
        kalman.resetINSError();
    }
}

void NavigationComputer::tryInitINS(double run_time) {
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

void NavigationComputer::predict(int predicts) {
    for (int i = 0; i < predicts; i++) {
        kalman.predict(ins->getState(), config.predict, config.T_kalman);
        kalman_time += config.T_kalman;
    }
}

void NavigationComputer::update() {
    Eigen::Matrix3d R_imu2nav = ins->getState().q_imu2nav.matrix();

    Eigen::Matrix<double, 11, 1> z;
    z.fill(0);
    bool a_valid = false;
    bool m_valid = false;
    bool d_valid_ = false;
    bool z_valid = false;

    // accelerometer hack, only activates if the y_a_log is full
    if (y_a_log.size() == config.y_a_log_size) {
        // compute mean of entire y_a_log
        Eigen::Vector3d y_a_mean = mean(Eigen::Vector3d::Zero(),
                                        y_a_log.begin(),
                                        y_a_log.end());
        // compute the error in magnitudes
        double y_a_norm_error = (y_a_mean - ins->getState().b_a).norm() - config.init.g_nav.norm();
        stats.y_a_norm_error = y_a_norm_error;

        // if the error is low enough, we assume there are no external accelerations
        // and apply the hack
        if (fabs(y_a_norm_error) < config.y_a_max_norm_error) {
            // only average data from the middle of the buffer, where
            // we are most certain there are no external accelerations
            Eigen::Vector3d y_a = mean(Eigen::Vector3d::Zero(),
                                       y_a_log.begin() + y_a_log.size()/4,
                                       y_a_log.begin() + 3*y_a_log.size()/4);
            z.segment<3>(0) = y_a - ins->getState().b_a -R_imu2nav.transpose()*ins->getState().g_nav;
            a_valid = true;
            stats.y_a_count++;
        }
    }

    if (y_m) {
        z.segment<3>(3) = *y_m - R_imu2nav.transpose()*config.update.m_nav;
        m_valid = true;
        stats.y_m_count++;
    }

    if (std::count(d_valid.begin(), d_valid.end(), true) > 0) {
        z.segment<4>(6) = y_d - config.update.beam_mat*(
            R_imu2nav.transpose()*ins->getState().v_nav +
            ins->getState().w_imu.cross(config.update.r_imu2dvl));
        d_valid_ = true;
        stats.y_d_count++;
    }

    if (y_z) {
        z[10] = *y_z - -(ins->getState().p_nav + R_imu2nav*config.update.r_imu2depth)[2];
        z_valid = true;
        stats.y_z_count++;
    }

    if (!a_valid && !m_valid && !d_valid_ && !z_valid) {
        return;
    }

    kalman.update(z, a_valid, m_valid, d_valid, z_valid, ins->getState(), config.update);

    y_m = boost::none;
    std::fill(d_valid.begin(), d_valid.end(), false);
    y_z = boost::none;
}

bool NavigationComputer::verifyKalmanTime(const std::string &sensor,
                                          double measurement_time) {
    double dt = measurement_time - kalman_time;

    if (dt < -config.T_kalman/2) {
        ROS_WARN_STREAM("Late " << sensor << " message, discarding " \
                        "(dt = " << dt);
        return false;
    } else if (dt > 2*config.T_kalman) {
        ROS_WARN_STREAM(sensor << " message originates from future, discarding " \
                        "(dt = " << dt);
        return false;
    } else {
        return true;
    }
}
