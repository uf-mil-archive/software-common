#include <iostream>

#include "INS.h"
#include "Kalman.h"

int main(int argc, char **argv) {
/*    INS ins(Eigen::Quaterniond(0, 1, 0, 0),
            Eigen::Vector3d(1, 2, 3),
            Eigen::Vector3d(4, 5, 6),
            Eigen::Vector3d(0, 0, -9.8));

    INS::Measurement measurement_prev = {
        Eigen::Vector3d(.1, .1, .1),
        Eigen::Vector3d(-.2, -.2, -.2)
    };

    INS::Measurement measurement = {
        Eigen::Vector3d(.11, .11, .11),
        Eigen::Vector3d(-.21, -.21, -.21)
    };

    ins.update(measurement_prev, .01);
    ins.update(measurement, .01);

    const INS::State &state = ins.getState();
    std::cout << state.p_nav << std::endl;*/

/*    Kalman kalman;
    kalman.x = Eigen::Matrix<double, 15, 1>::Ones();
    kalman.P = Eigen::Matrix<double, 15, 15>::Ones() + 10*Eigen::Matrix<double, 15, 15>::Identity();

    INS::State state;
    state.q_imu2nav = Eigen::Quaterniond(1, 2, 3, 4).normalized();
    state.w_imu = Eigen::Vector3d(1, 2, 3);
    state.a_imu = Eigen::Vector3d(-1, -3, 5);
    state.g_nav = Eigen::Vector3d(0, 0, -9.8);

    Kalman::PredictConfig predict;
    predict.R_g = Eigen::DiagonalMatrix<double, 3, 3>(
        4.6254e-6, 4.6254e-6, 4.6254e-6);
    predict.R_a = Eigen::DiagonalMatrix<double, 3, 3>(
        1.1413e-6, 1.1413e-6, 1.1413e-6);
    predict.Q_b_g = Eigen::DiagonalMatrix<double, 3, 3>(
        1.2217e-8, 1.2217e-8, 1.2217e-8);
    predict.Q_b_a = Eigen::DiagonalMatrix<double, 3, 3>(
        1.9700e-7, 1.9700e-7, 1.9700e-7);

    kalman.predict(state, predict, .01);

    std::cout << kalman.x << std::endl;
    std::cout << kalman.P << std::endl;*/

    Kalman kalman;
    kalman.x = Eigen::Matrix<double, 15, 1>::Ones();
    kalman.P = 1e-6*Eigen::Matrix<double, 15, 15>::Ones() + 10*Eigen::Matrix<double, 15, 15>::Identity();

    INS::State state;
    state.q_imu2nav = Eigen::Quaterniond(1, 2, 3, 4).normalized();
    state.w_imu = Eigen::Vector3d(1, 2, 3);
    state.a_imu = Eigen::Vector3d(-1, -3, 5);
    state.g_nav = Eigen::Vector3d(0, 0, -9.8);
    state.v_nav = Eigen::Vector3d(2, 0, 0);

    Kalman::UpdateConfig update;
    update.R_g = Eigen::DiagonalMatrix<double, 3, 3>(
        4.6254e-6, 4.6254e-6, 4.6254e-6);
    update.R_a = Eigen::DiagonalMatrix<double, 3, 3>(
        1.1413e-04, 1.1413e-04, 1.1413e-04);
    update.R_m = Eigen::DiagonalMatrix<double, 3, 3>(
        1e-6, 1e-6, 1e-6);
    update.R_d.fill(0);
    for (int i=0; i<4; i++)
        update.R_d(i, i) = 4.9e-5;
    update.R_z = 3e-3;

    update.beam_mat <<
        0.5, 0, 0.86603,
        -0.5, 0, 0.86603,
        0, -0.5, 0.86603,
        0, 0.5, 0.86603;
    update.r_imu2dvl <<
        0, 0, -0.102-0.076;
    update.q_imu2dvl = Eigen::Quaterniond(
        0.000284, 0.394308, -0.918965, -0.005013);
    update.r_imu2depth <<
        0.445 - 0.431, 0.102, -0.051 - 0.076;
    update.m_nav <<
        -2244.2, 24151.0, -40572.8;

    Eigen::Matrix<double, 11, 1> z;
    z.fill(2);

    kalman.update(z, true, true, {{true, true, true, true}}, true, state, update);

//    std::cout << kalman.x << std::endl;
//    std::cout << kalman.P << std::endl;
}
