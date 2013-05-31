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
        // real sub
        //config.T_imu = 1 / 204.0;
        //config.T_kalman = 1 / 50.0;
//        config.y_a_max_norm_error = 2e-3;
//        config.y_a_log_size = 20;

        // sim
        config.T_imu = 1 / 30.0;
        config.T_kalman = 1 / 15.0;
        config.y_a_max_norm_error = 5e-3;
        config.y_a_log_size = 5;

        config.predict.R_g = Eigen::DiagonalMatrix<double, 3, 3>(
            5e-5, 5e-5, 5e-5);
        config.predict.R_a = Eigen::DiagonalMatrix<double, 3, 3>(
            3e-3, 3e-3, 3e-3);
        config.predict.Q_b_g = Eigen::DiagonalMatrix<double, 3, 3>(
            1e-10, 1e-10, 1e-10);
        config.predict.Q_b_a = Eigen::DiagonalMatrix<double, 3, 3>(
            1e-10, 1e-10, 1e-10);

        config.update.R_g = config.predict.R_g;
        config.update.R_a = 10*config.predict.R_a;
        config.update.R_m = Eigen::DiagonalMatrix<double, 3, 3>(
            4e-14, 4e-14, 4e-14);
        config.update.R_d.fill(0);
        for (int i=0; i<4; i++)
            config.update.R_d(i, i) = 0.0004;
        config.update.R_z = 4e-5;
        config.update.m_nav <<
            -2244.2e-9, 24151.0e-9, -40572.8e-9;

        tf::StampedTransform imu2dvl;
        tf_listener.waitForTransform("/dvl", "/imu", ros::Time::now(), ros::Duration(10.0));
        tf_listener.lookupTransform("/dvl", "/imu", ros::Time::now(), imu2dvl);
        tf::StampedTransform imu2depth;
        tf_listener.waitForTransform("/depth", "/imu", ros::Time::now(), ros::Duration(10.0));
        tf_listener.lookupTransform("/depth", "/imu", ros::Time::now(), imu2depth);

        Eigen::Matrix<double, 4, 3> beam_mat_dvl;
        beam_mat_dvl <<
            -0.5, 0, -sqrt(3)/2,
            0.5, 0, -sqrt(3)/2,
            0, 0.5, -sqrt(3)/2,
            0, -0.5, -sqrt(3)/2;
        Eigen::Quaterniond q_imu2dvl = uf_common::quat2quat(imu2dvl.getRotation());
        config.update.beam_mat = beam_mat_dvl * q_imu2dvl.matrix();

        config.update.r_imu2dvl = uf_common::vec2vec(imu2dvl.getOrigin());
        config.update.r_imu2depth = uf_common::vec2vec(imu2depth.getOrigin());

        config.init.accel_samples = 30;
        config.init.mag_samples = 30;
        config.init.dvl_samples = 5;
        config.init.depth_samples = 10;
        config.init.g_nav = Eigen::Vector3d(0, 0, -9.81);
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
                state->filt.q_imu2nav);
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
                state->filt.q_imu2nav);
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
