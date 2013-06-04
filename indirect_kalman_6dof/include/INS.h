#ifndef INS_H
#define INS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/optional.hpp>

class INS {
public:
    // Creates an INS at the initial state estimate
    INS(const Eigen::Quaterniond &init_q_imu2nav,
        const Eigen::Vector3d &init_v_nav,
        const Eigen::Vector3d &init_p_nav,
        const Eigen::Vector3d &g_nav);

    // INS errors
    struct Error {
        Eigen::Quaterniond q;
        Eigen::Vector3d v_nav;
        Eigen::Vector3d p_nav;
        Eigen::Vector3d b_g;
    };

    // INS state variables
    struct State {
        Eigen::Quaterniond q_imu2nav;
        Eigen::Vector3d v_nav;
        Eigen::Vector3d p_nav;

        Eigen::Vector3d w_imu;
        Eigen::Vector3d a_imu;

        Eigen::Vector3d b_g;
        Eigen::Vector3d g_nav;

        void correct(const Error &error);
    };

    const State &getState() const { return state; }

    // Updates the INS from a new inertial measurement
    struct Measurement {
        Eigen::Vector3d y_a;
        Eigen::Vector3d y_g;
    };
    void update(const Measurement &cur, double dt);

    // corrects INS for errors
    void correct(const Error &error) { state.correct(error); }

private:
    State state;
    boost::optional<Measurement> prev_measurement;
};

#endif
