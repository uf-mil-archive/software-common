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
    state.ins_time = ins_time;
    return state;
}

void NavigationComputer::updateINS(const INS::Measurement &measurement,
                                   double measurement_time,
                                   double now_time) {
    if (!ins) {
        ins_init.updateAccel(measurement.y_a);
        ins_time = measurement_time;
        tryInitINS();
        return;
    }

    // Calculate how many updates are necessary to bring INS to this
    // measurement.
    double T_delta;
    if (config.verify_timestamps) {
        T_delta = measurement_time - ins_time;
    } else {
        T_delta = config.T_imu;
    }

    int updates = static_cast<int>(round(T_delta / config.T_imu));

    if (updates <= 0) {
        ROS_WARN_STREAM("Too frequent IMU measusrements, dropping"
                        "(T_delta " << T_delta << " T_imu " << config.T_imu << ")");
    }

    if (updates >= 5) {
        // too many dropped messages, just advance time.
        ROS_WARN_STREAM("Dropped or too infrequent IMU measurements, advancing time "
                        "(T_delta " << T_delta << " T_imu " << config.T_imu << ")");
        ins_time += (updates-1)*config.T_imu;
        updates = 1;
    }

    if (updates >= 2) {
        ROS_WARN_STREAM("Performing " << updates << " updates to compensate for dropped or infrequent INS messages");
    }

    // Run the correct number of INS updates
    for (int i=0; i<updates; i++) {
        ins->update(measurement, config.T_imu);
        ins_time += config.T_imu;
    }

    // If we're not relying on timestamps, keep ins_time synced with
    // current time
    if (!config.verify_timestamps) {
        ins_time = now_time;
    }
}

void NavigationComputer::updateMag(const Eigen::Vector3d &y_m,
                                   double measurement_time) {
    // if we don't have an INS, forward to the INSInitializer
    if (!ins) {
        ins_init.updateMag(y_m);
        tryInitINS();
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
        tryInitINS();
        return;
    }

    if (!verifyKalmanTime("depth", measurement_time)) {
        return;
    }

    this->y_z = y_z;
}

void NavigationComputer::updateKalman() {
    if (!ins) {
        return;
    }

    // compute how many predictions are necessary to bring kalman
    // even or slightly past the INS
    double T_delta = ins_time - kalman_time;
    int predicts = static_cast<int>(round(T_delta / config.T_kalman));

    // Not yet time for a predict, so quit
    if (predicts == 0) {
        return;
    }

    if (predicts > 5) {
        ROS_WARN_STREAM("Running " << predicts << " kalman predictions to catch up to INS time");
    } else if (predicts > 1) {
        ROS_DEBUG_STREAM("Running " << predicts << " kalman predictions to catch up to INS time");
    }

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

void NavigationComputer::tryInitINS() {
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
    kalman_time = ins_time;
    last_correction_time = ins_time;
}

void NavigationComputer::predict(int predicts) {
    for (int i = 0; i < predicts; i++) {
        kalman.predict(ins->getState(), config.predict, config.T_kalman);
        kalman_time += config.T_kalman;
    }
}

void NavigationComputer::update() {
    Eigen::Matrix3d R_imu2nav = ins->getState().q_imu2nav.matrix();

    Kalman::MeasurementVec z;
    z.fill(0);
    bool m_valid = false;
    bool d_valid_ = false;
    bool z_valid = false;

    if (y_m) {
        z.segment<3>(0) = *y_m - R_imu2nav.transpose()*config.update.m_nav;
        m_valid = true;
        stats.y_m_count++;
    }

    if (std::count(d_valid.begin(), d_valid.end(), true) > 0) {
        z.segment<4>(3) = y_d - config.update.beam_mat*(
            R_imu2nav.transpose()*ins->getState().v_nav +
            ins->getState().w_imu.cross(config.update.r_imu2dvl));
        d_valid_ = true;
        stats.y_d_count++;
    }

    if (y_z) {
        z[7] = *y_z - -(ins->getState().p_nav + R_imu2nav*config.update.r_imu2depth)[2];
        z_valid = true;
        stats.y_z_count++;
    }

    if (!m_valid && !d_valid_ && !z_valid) {
        return;
    }

    kalman.update(z, m_valid, d_valid, z_valid, ins->getState(), config.update);

    y_m = boost::none;
    std::fill(d_valid.begin(), d_valid.end(), false);
    y_z = boost::none;
}

bool NavigationComputer::verifyKalmanTime(const std::string &sensor,
                                          double measurement_time) {
    if (!config.verify_timestamps) {
        return true;
    }

    double dt = measurement_time - kalman_time;

    if (dt < -config.T_kalman/2) {
        ROS_WARN_STREAM("Late " << sensor << " message, discarding " \
                        "(dt = " << dt);
        return false;
    } else if (dt > 3.0/2*config.T_kalman) {
        ROS_WARN_STREAM("Future " << sensor << " message, discarding " \
                        "(dt = " << dt);
        return false;
    }

    return true;
}
