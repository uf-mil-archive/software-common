#include "NavigationComputer.h"
#include "indirect_kalman_6dof/Debug.h"

#include <uf_common/param_helpers.h>
#include <uf_common/msg_helpers.h>
#include <uf_common/Float64Stamped.h>
#include <uf_common/VelocityMeasurements.h>

#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

struct Node {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber imu_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber dvl_sub;
    tf::TransformListener tf_listener;
    ros::Timer update_timer;
    ros::Publisher odometry_pub;
    ros::Publisher pose_pub;
    ros::Publisher debug_pub;

    boost::scoped_ptr<NavigationComputer> navcomp;

    Node() :
        private_nh("~")
    {
        // TODO read from config
        // TODO come up with good values
        NavigationComputer::Config config;
        //config.T_imu = 1 / 204.0;
        config.T_imu = 1 / 30.0;
        config.T_kalman = 1 / 30.0;
        config.T_kalman_correction = 2.0;

        config.predict.R_g = Eigen::DiagonalMatrix<double, 3, 3>(
            4.6254e-6, 4.6254e-6, 4.6254e-6);
        config.predict.R_a = Eigen::DiagonalMatrix<double, 3, 3>(
            1.1413e-6, 1.1413e-6, 1.1413e-6);
        config.predict.Q_b_g = Eigen::DiagonalMatrix<double, 3, 3>(
            1.2217e-8, 1.2217e-8, 1.2217e-8);
        config.predict.Q_b_a = Eigen::DiagonalMatrix<double, 3, 3>(
            1.9700e-7, 1.9700e-7, 1.9700e-7);

        config.update.R_g = config.predict.R_g;
        config.update.R_a = 100*config.predict.R_a;
        config.update.R_m = Eigen::DiagonalMatrix<double, 3, 3>(
            1e-6, 1e-6, 1e-6);
        config.update.R_d.fill(0);
        for (int i=0; i<4; i++)
            config.update.R_d(i, i) = 4.9e-5;
        config.update.R_z = 5e-3;

/*        config.update.beam_mat <<
            0.5, 0, 0.86603,
            -0.5, 0, 0.86603,
            0, -0.5, 0.86603,
            0, 0.5, 0.86603;*/
        config.update.beam_mat <<
            0.35355, -0.35355, 0.866025,
            -0.35355, 0.35355, 0.866025,
            -0.35355, -0.35355, 0.866025,
            0.35355, 0.35355, 0.866025;

        // TODO these r_imu2xxx probably need to be rotated 180 degrees about X
        config.update.r_imu2dvl <<
            0, 0, -0.102-0.076;
        config.update.q_imu2dvl = Eigen::Quaterniond(
            0.000284, 0.394308, -0.918965, -0.005013);
        config.update.r_imu2depth <<
            0.445 - 0.431, 0.102, -0.051 - 0.076;
        config.update.m_nav <<
            -2244.2, 24151.0, -40572.8;

        config.init.accel_samples = 50;
        config.init.mag_samples = 50;
        config.init.dvl_samples = 5;
        config.init.depth_samples = 10;
        config.init.g_nav = Eigen::Vector3d(0, 0, -9.8);
        config.init.m_nav = config.update.m_nav;
        config.init.r_imu2depth = config.update.r_imu2depth;
        config.init.beam_mat = config.update.beam_mat;

        navcomp.reset(new NavigationComputer(config));

        imu_sub = nh.subscribe("imu/data_raw", 1, &Node::onImu, this);
        mag_sub = nh.subscribe("imu/mag", 1, &Node::onMag, this);
        depth_sub = nh.subscribe("depth", 1, &Node::onDepth, this);
        dvl_sub = nh.subscribe("dvl", 1, &Node::onDvl, this);
        update_timer = nh.createTimer(ros::Duration(config.T_kalman), &Node::onUpdate, this);
        odometry_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
        debug_pub = nh.advertise<indirect_kalman_6dof::Debug>("debug", 1);
    }

    void onImu(sensor_msgs::ImuConstPtr imu) {
        INS::Measurement measurement = {
            uf_common::xyz2vec(imu->linear_acceleration),
            uf_common::xyz2vec(imu->angular_velocity)
        };

        navcomp->updateINS(measurement, imu->header.stamp.toSec());
    }

    void onMag(geometry_msgs::Vector3StampedConstPtr mag) {
        // TODO filter messages that are too old
        Eigen::Vector3d y_m = uf_common::xyz2vec(mag->vector);
        navcomp->updateMag(y_m);
    }

    void onDepth(uf_common::Float64StampedConstPtr depth) {
        double y_z = depth->data;
        navcomp->updateDepth(y_z);
    }

    void onDvl(uf_common::VelocityMeasurementsConstPtr vels) {
        Eigen::Matrix<double, 4, 1> y_d;
        boost::array<bool, 4> d_valid = {{true, true, true, true}};

        for (int i=0; i<4; i++) {
            y_d[i] = vels->velocity_measurements[i].velocity;
            if (y_d[i] != y_d[i]) {
                y_d[i] = 0;
                d_valid[i] = false;
            }
        }

        navcomp->updateDVL(y_d, d_valid);
    }

    void onUpdate(const ros::TimerEvent &event) {
        // TODO use timestamp from kalman instead of timer
        // TODO don't hardcode frames
        navcomp->run(event.current_real.toSec());

        boost::optional<NavigationComputer::State> state = navcomp->getState();
        if (state) {
            nav_msgs::Odometry msg;
            msg.header.stamp = event.current_real;
            msg.header.frame_id = "/map";
            msg.child_frame_id = "/imu";
            msg.pose.pose.position = uf_common::vec2xyz<geometry_msgs::Point>(
                state->filt.p_nav);
            msg.pose.pose.orientation = uf_common::quat2xyzw<geometry_msgs::Quaternion>(
                state->filt.q_imu2nav.conjugate());
            msg.twist.twist.linear = uf_common::vec2xyz<geometry_msgs::Vector3>(
                state->filt.q_imu2nav.matrix().transpose() * state->filt.v_nav);
            msg.twist.twist.angular = uf_common::vec2xyz<geometry_msgs::Vector3>(
                state->filt.w_imu);
            odometry_pub.publish(msg);

            geometry_msgs::PoseStamped posemsg;
            posemsg.header.stamp = event.current_real;
            posemsg.header.frame_id = "/map";
            posemsg.pose.position = uf_common::vec2xyz<geometry_msgs::Point>(
                state->filt.p_nav);
            posemsg.pose.orientation = uf_common::quat2xyzw<geometry_msgs::Quaternion>(
                state->filt.q_imu2nav.conjugate());
            pose_pub.publish(posemsg);

            indirect_kalman_6dof::Debug debugmsg;
            debugmsg.header.stamp = event.current_real;
            debugmsg.b_g = uf_common::vec2xyz<geometry_msgs::Vector3>(
                state->filt.b_g);
            debugmsg.b_a = uf_common::vec2xyz<geometry_msgs::Vector3>(
                state->filt.b_a);
            debugmsg.a_imu = uf_common::vec2xyz<geometry_msgs::Vector3>(
                state->filt.a_imu);
            debugmsg.w_imu = uf_common::vec2xyz<geometry_msgs::Vector3>(
                state->filt.w_imu);
            debug_pub.publish(debugmsg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "indirect_kalman_6dof");

    Node n;
    ros::spin();
    return 0;
}
