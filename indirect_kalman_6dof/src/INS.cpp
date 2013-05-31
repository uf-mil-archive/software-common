#include "INS.h"

INS::INS(const Eigen::Quaterniond &init_q_imu2nav,
         const Eigen::Vector3d &init_v_nav,
         const Eigen::Vector3d &init_p_nav,
         const Eigen::Vector3d &g_nav) {
    state.q_imu2nav = init_q_imu2nav;
    state.v_nav = init_v_nav;
    state.p_nav = init_p_nav;
    state.w_imu.fill(0);
    state.a_imu.fill(0);
    state.b_g.fill(0);
    state.b_a.fill(0);
    state.g_nav = g_nav;
}

static Eigen::Quaterniond vec2quat(const Eigen::Vector3d &vec) {
    return Eigen::Quaterniond(0.0, vec.x(), vec.y(), vec.z());
}

static Eigen::Quaterniond operator+(const Eigen::Quaterniond &a,
                                    const Eigen::Quaterniond &b) {
    return Eigen::Quaterniond(a.w()+b.w(), a.x()+b.x(), a.y()+b.y(), a.z()+b.z());
}

static Eigen::Quaterniond &operator+=(Eigen::Quaterniond &a,
                                      const Eigen::Quaterniond &b) {
    a = a + b;
    return a;
}

static Eigen::Quaterniond operator*(double s,
                                    const Eigen::Quaterniond &q) {
    return Eigen::Quaterniond(s*q.w(), s*q.x(), s*q.y(), s*q.z());
}

void INS::update(const Measurement &cur, double dt) {
    // If we don't have a previous measurement, save this one and stop
    if (!prev_measurement) {
        prev_measurement = cur;
        return;
    }
    Measurement &prev = *prev_measurement;

    // Integrate w_imu to q_imu2nav
    Eigen::Quaterniond q_imu2nav_prev = state.q_imu2nav;
    Eigen::Vector3d w_imu_prev = state.w_imu;
    state.w_imu = cur.y_g - state.b_g;
    Eigen::Vector3d w_imu_mid = (state.w_imu + w_imu_prev)/2;
    {
        Eigen::Quaterniond k1 = 0.5 * (state.q_imu2nav * vec2quat(w_imu_prev));
        Eigen::Quaterniond k2 = 0.5 * (state.q_imu2nav + dt/2*k1) * vec2quat(w_imu_mid);
        Eigen::Quaterniond k3 = 0.5 * (state.q_imu2nav + dt/2*k2) * vec2quat(w_imu_mid);
        Eigen::Quaterniond k4 = 0.5 * (state.q_imu2nav + dt*k3) * vec2quat(state.w_imu);
        state.q_imu2nav += dt/6*(k1 + 2*k2 + 2*k3 + k4);
        state.q_imu2nav.normalize();
    }

    // Integrate a_imu to v_nav
    Eigen::Vector3d v_nav_prev = state.v_nav;
    Eigen::Vector3d a_imu_prev = state.a_imu;
    state.a_imu = cur.y_a - state.b_a;
    Eigen::Vector3d a_imu_mid = (state.a_imu + a_imu_prev)/2;
    Eigen::Quaterniond q_imu2nav_mid = (1/2.0) * (state.q_imu2nav + q_imu2nav_prev);
    q_imu2nav_mid.normalize();
    {
        Eigen::Vector3d k1 = q_imu2nav_prev.matrix() * a_imu_prev + state.g_nav;
        Eigen::Vector3d k23 = q_imu2nav_mid.matrix() * a_imu_mid + state.g_nav;
        Eigen::Vector3d k4 = state.q_imu2nav.matrix() * state.a_imu + state.g_nav;
        state.v_nav += dt/6*(k1 + 4*k23 + k4);
    }

    // Integrate v_nav to p_nav
    state.p_nav += dt/2*(v_nav_prev + state.v_nav);

    // Save current measurement as previous
    prev = cur;
}

void INS::State::correct(const Error &error) {
    q_imu2nav = q_imu2nav * error.q;
    q_imu2nav.normalize();
    v_nav += error.v_nav;
    p_nav += error.p_nav;
    w_imu -= error.b_g;
    a_imu -= error.b_a;
    b_g += error.b_g;
    b_a += error.b_a;
}
