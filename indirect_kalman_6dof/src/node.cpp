#include "NavigationComputer.h"
#include "indirect_kalman_6dof/Debug.h"
#include "indirect_kalman_6dof/SetPosition.h"
#include "indirect_kalman_6dof/SetIgnoreMagnetometer.h"

#include <uf_common/param_helpers.h>
#include <uf_common/msg_helpers.h>
#include <uf_common/Float64Stamped.h>
#include <uf_common/VelocityMeasurements.h>

#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

static Eigen::Matrix3d vec2diag(const Eigen::Vector3d &x) {
    return Eigen::DiagonalMatrix<double, 3, 3>(x[0], x[1], x[2]);
}

struct Node {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber imu_sub;
    ros::Subscriber mag_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber dvl_sub;
    tf::TransformListener tf_listener;
    ros::Timer pub_timer;
    ros::Publisher odometry_pub;
    ros::Publisher pose_pub;
    ros::Publisher debug_pub;
    ros::ServiceServer set_position_srv;
    ros::ServiceServer set_ignore_magnetometer_srv;
    bool ignoreMagnetometer;

    boost::scoped_ptr<NavigationComputer> navcomp;

    Node() :
        private_nh("~"), ignoreMagnetometer(false)
    {
        NavigationComputer::Config config;

        double f_imu = uf_common::getParam<double>(private_nh, "f_imu");
        config.T_imu = 1 / f_imu;

        double f_kalman = uf_common::getParam<double>(private_nh, "f_kalman");
        config.T_kalman = 1 / f_kalman;

        config.verify_timestamps = uf_common::getParam<bool>(private_nh, "verify_timestamps", true);
        if (!config.verify_timestamps) {
            ROS_WARN("Not verifying timestamps on sensor data.");
        }

        config.predict.R_g = vec2diag(uf_common::getParam<Eigen::Vector3d>(private_nh, "R_g"));
        config.predict.R_a = vec2diag(uf_common::getParam<Eigen::Vector3d>(private_nh, "R_a"));
        config.predict.Q_b_g = vec2diag(uf_common::getParam<Eigen::Vector3d>(private_nh, "Q_b_g"));

        config.update.R_g = config.predict.R_g;
        config.update.R_m = vec2diag(uf_common::getParam<Eigen::Vector3d>(private_nh, "R_m"));
        double R_d = uf_common::getParam<double>(private_nh, "R_d");
        config.update.R_d.fill(0);
        for (int i=0; i<4; i++)
            config.update.R_d(i, i) = R_d;
        config.update.R_z = uf_common::getParam<double>(private_nh, "R_z");

        config.update.m_nav = uf_common::getParam<Eigen::Vector3d>(private_nh, "m_nav");

        tf::StampedTransform imu2dvl;
        tf_listener.waitForTransform("/dvl", "/imu", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform("/dvl", "/imu", ros::Time(0), imu2dvl);
        tf::StampedTransform imu2depth;
        tf_listener.waitForTransform("/depth", "/imu", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform("/depth", "/imu", ros::Time(0), imu2depth);

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

        config.init.accel_samples = uf_common::getParam<unsigned int>(private_nh, "init_accel_samples");
        config.init.mag_samples = uf_common::getParam<unsigned int>(private_nh, "init_mag_samples");
        config.init.dvl_samples = uf_common::getParam<unsigned int>(private_nh, "init_dvl_samples");
        config.init.depth_samples = uf_common::getParam<unsigned int>(private_nh, "init_depth_samples");
        config.init.g_nav = uf_common::getParam<Eigen::Vector3d>(private_nh, "g_nav");
        config.init.m_nav = config.update.m_nav;
        config.init.r_imu2depth = config.update.r_imu2depth;
        config.init.beam_mat = config.update.beam_mat;

        navcomp.reset(new NavigationComputer(config));

        imu_sub = nh.subscribe("imu/data_raw", 1, &Node::onImu, this);
        mag_sub = nh.subscribe("imu/mag", 1, &Node::onMag, this);
        depth_sub = nh.subscribe("depth", 1, &Node::onDepth, this);
        dvl_sub = nh.subscribe("dvl", 1, &Node::onDvl, this);
        pub_timer = nh.createTimer(ros::Duration(config.T_kalman), &Node::onPub, this);
        odometry_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
        debug_pub = private_nh.advertise<indirect_kalman_6dof::Debug>("debug", 1);
        set_position_srv = private_nh.advertiseService("set_position", &Node::onSetPosition, this);
        set_ignore_magnetometer_srv = private_nh.advertiseService("set_ignore_magnetometer", &Node::onSetIgnoreMagnetometer, this);
    }

    void onImu(sensor_msgs::ImuConstPtr imu) {
        INS::Measurement measurement = {
            uf_common::xyz2vec(imu->linear_acceleration),
            uf_common::xyz2vec(imu->angular_velocity)
        };

        navcomp->updateINS(measurement, imu->header.stamp.toSec(), ros::Time::now().toSec());
        navcomp->updateKalman();
    }

    void onMag(sensor_msgs::MagneticFieldConstPtr mag) {
        if(ignoreMagnetometer) return;
        Eigen::Vector3d y_m = uf_common::xyz2vec(mag->magnetic_field);
        navcomp->updateMag(y_m, mag->header.stamp.toSec());
    }

    void onDepth(uf_common::Float64StampedConstPtr depth) {
        double y_z = depth->data;
        navcomp->updateDepth(y_z, depth->header.stamp.toSec());
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

        navcomp->updateDVL(y_d, d_valid, vels->header.stamp.toSec());
    }

    void onPub(const ros::TimerEvent &event) {
        boost::optional<NavigationComputer::State> state = navcomp->getState();
        if (!state) {
            return;
        }

        Eigen::Matrix3d R_nav2imu = state->filt.q_imu2nav.matrix().transpose();

        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time(state->ins_time);
        msg.header.frame_id = "/map";
        msg.child_frame_id = "/imu";
        msg.pose.pose.position = uf_common::vec2xyz<geometry_msgs::Point>(
            state->filt.p_nav);
        msg.pose.pose.orientation = uf_common::quat2xyzw<geometry_msgs::Quaternion>(
            state->filt.q_imu2nav);
        msg.twist.twist.linear = uf_common::vec2xyz<geometry_msgs::Vector3>(
            R_nav2imu * state->filt.v_nav);
        msg.twist.twist.angular = uf_common::vec2xyz<geometry_msgs::Vector3>(
            state->filt.w_imu);
        odometry_pub.publish(msg);

        geometry_msgs::PoseStamped posemsg;
        posemsg.header.stamp = ros::Time(state->ins_time);
        posemsg.header.frame_id = "/map";
        posemsg.pose.position = uf_common::vec2xyz<geometry_msgs::Point>(
            state->filt.p_nav);
        posemsg.pose.orientation = uf_common::quat2xyzw<geometry_msgs::Quaternion>(
            state->filt.q_imu2nav);
        pose_pub.publish(posemsg);

        indirect_kalman_6dof::Debug debugmsg;
        debugmsg.header.stamp = ros::Time(state->ins_time);
        debugmsg.b_g = uf_common::vec2xyz<geometry_msgs::Vector3>(
            state->filt.b_g);
        debugmsg.a_imu = uf_common::vec2xyz<geometry_msgs::Vector3>(
            state->filt.a_imu);
        debugmsg.a_imu_no_g = uf_common::vec2xyz<geometry_msgs::Vector3>(
            state->filt.a_imu + R_nav2imu * state->filt.g_nav);
        debugmsg.w_imu = uf_common::vec2xyz<geometry_msgs::Vector3>(
            state->filt.w_imu);
        debugmsg.y_m_count = state->stats.y_m_count;
        debugmsg.y_d_count = state->stats.y_d_count;
        debugmsg.y_z_count = state->stats.y_z_count;
        debug_pub.publish(debugmsg);
    }

    bool onSetPosition(indirect_kalman_6dof::SetPosition::Request &request,
                       indirect_kalman_6dof::SetPosition::Response &response) {
        return navcomp->setPosition(uf_common::xyz2vec(request.position));
    }
    
    bool onSetIgnoreMagnetometer(indirect_kalman_6dof::SetIgnoreMagnetometer::Request &request,
                                 indirect_kalman_6dof::SetIgnoreMagnetometer::Response &response) {
        ignoreMagnetometer = request.ignore;
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "indirect_kalman_6dof");

    Node n;
    ros::spin();
    return 0;
}
