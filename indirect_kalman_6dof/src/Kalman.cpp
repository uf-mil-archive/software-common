#include "Kalman.h"
#include <Eigen/LU>

#include <ros/ros.h>
#include <iomanip>

Kalman::Kalman() {
    reset();
}

namespace {
    Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
        return (Eigen::Matrix3d() <<
                0, -v[2], v[1],
                v[2], 0, -v[0],
                -v[1], v[0], 0).finished();
    }
}

INS::Error Kalman::getINSError() const {
    INS::Error error;
    error.q = Eigen::Quaterniond(1, x[0], x[1], x[2]);
    error.v_nav = x.segment<3>(3);
    error.p_nav = x.segment<3>(6);
    error.b_g = x.segment<3>(9);
    return error;
}

void Kalman::resetINSError() {
    x.fill(0);
}

void Kalman::reset() {
    resetINSError();
    P.fill(0);
    for (int i=0; i<6; i++)
        P(i, i) = 1e-4;
    P(8, 8) = 1e-2;
    for (int i=9; i<12; i++)
        P(i, i) = 1e-3;
}

void Kalman::predict(const INS::State &ins, const PredictConfig &config, double dt) {
    using Eigen::Matrix;
    using Eigen::Matrix3d;

    Matrix3d R_imu2nav = ins.q_imu2nav.matrix();

    StateMat A;
    A.fill(0);
    A.block<3, 3>(0, 0) = -skew(ins.w_imu) + .5*skew(x.segment<3>(9));
    A.block<3, 3>(0, 9) = -.5 * (Matrix3d::Identity() + skew(x.segment<3>(0)));
    A.block<3, 3>(3, 0) = -2*R_imu2nav*skew(ins.a_imu);
    A.block<3, 3>(6, 3) = Matrix3d::Identity();

    Matrix<double, 12, 9> L;
    L.fill(0);
    L.block<3, 3>(0, 0) = -0.5*(Matrix3d::Identity() + skew(x.segment<3>(0)));
    L.block<3, 3>(3, 3) = -R_imu2nav - 2*R_imu2nav*skew(x.segment<3>(0));
    L.block<3, 3>(9, 6) = Matrix3d::Identity();

    StateMat F_d;
    F_d = StateMat::Identity() + A*dt + 0.5*A*A*dt*dt;

    Matrix<double, 9, 9> Q;
    Q.fill(0);
    Q.block<3, 3>(0, 0) = config.R_g;
    Q.block<3, 3>(3, 3) = config.R_a;
    Q.block<3, 3>(6, 6) = config.Q_b_g;

    StateMat LQL = L*Q*L.transpose();

    StateMat Q_d;
    Q_d = LQL*dt +
        (dt*dt/2.0)*(A*LQL + LQL*A.transpose()) +
        (dt*dt*dt/3.0)*A*LQL*A.transpose();

    x = F_d*x;
    P = F_d*P*F_d.transpose() + Q_d;
}

void Kalman::update(const MeasurementVec &z,
                    bool m_valid,
                    const boost::array<bool, 4> &d_valid,
                    bool z_valid,
                    const INS::State &ins,
                    const UpdateConfig &config) {
    using Eigen::Matrix;
    using Eigen::Matrix3d;
    using Eigen::Matrix4d;
    using Eigen::Vector3d;
    using Eigen::RowVector3d;

    Matrix3d R_imu2nav = ins.q_imu2nav.matrix();

    Matrix<double, 8, 12> H;
    H.fill(0);
    H.block<3, 3>(0, 0) = 2*skew(R_imu2nav.transpose()*config.m_nav);
    H.block<4, 3>(3, 0) = 2*config.beam_mat*skew(R_imu2nav.transpose()*(
                                                     ins.v_nav + x.segment<3>(3)));
    H.block<4, 3>(3, 3) = config.beam_mat*R_imu2nav.transpose()
                          - 2*config.beam_mat*skew(x.segment<3>(0))*R_imu2nav.transpose();
    H.block<4, 3>(3, 9) = config.beam_mat*skew(config.r_imu2dvl);
    H.block<1, 3>(7, 0) = Eigen::RowVector3d(0, 0, -2)*R_imu2nav*skew(config.r_imu2depth);
    H(7, 8) = -1;

    if (!m_valid) {
        H.block<3, 12>(0, 0).fill(0);
    }

    for (int beam = 0; beam < 4; beam++) {
        if (!d_valid[beam]) {
            H.row(3 + beam).fill(0);
        }
    }

    if (!z_valid) {
        H.row(7).fill(0);
    }

    Matrix<double, 8, 11> M;
    M.fill(0);
    M.block<3, 3>(0, 3) = Matrix3d::Identity();
    M.block<4, 3>(3, 0) = config.beam_mat * skew(config.r_imu2dvl);
    M.block<4, 4>(3, 6) = Matrix4d::Identity();
    M(7, 10) = 1;

    Matrix<double, 11, 11> R;
    R.fill(0);
    R.block<3, 3>(0, 0) = config.R_g;
    R.block<3, 3>(3, 3) = config.R_m;
    R.block<4, 4>(6, 6) = config.R_d;
    R(10, 10) = config.R_z;

    Matrix<double, 8, 8> MRM = M*R*M.transpose();

    // solve for K using an Cholesky decomposition
    // K = PH'(HPH' + MRM')^-1
    // K(HPH' + MRM') = PH'
    // (HPH' + MRM')'K' = HP'
    // AX = B
    Matrix<double, 8, 8> Kt_A = (H*P*H.transpose() + MRM).transpose();
    Matrix<double, 8, 12> Kt_B = H*P.transpose();
    Matrix<double, 12, 8> K = Kt_A.ldlt().solve(Kt_B).transpose();

    Vector3d yaw_imu = R_imu2nav.transpose().col(2);
    Matrix<double, 3, 3> limit_yaw = yaw_imu * yaw_imu.transpose();
    K.block<3, 3>(0, 0) = limit_yaw * K.block<3, 3>(0, 0);
    K.block<6, 3>(3, 0).fill(0);
    K.block<3, 3>(9, 0) = limit_yaw * K.block<3, 3>(9, 0);

    x += K*(z - H*x);
    StateMat tmp = StateMat::Identity() - K*H;
    P = tmp*P*tmp.transpose() + K*MRM*K.transpose();
}
