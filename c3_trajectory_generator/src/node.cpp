#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include <uf_common/PoseTwistStamped.h>
#include <uf_common/msg_helpers.h>
#include <uf_common/param_helpers.h>
#include <kill_handling/Kill.h>
#include <kill_handling/listener.h>

#include "uf_common/MoveToAction.h"
#include "c3_trajectory_generator/SetDisabled.h"
#include "C3Trajectory.h"

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace uf_common;
using namespace c3_trajectory_generator;

// take a Post and Twist object and translate it to C3 'Point' object
subjugator::C3Trajectory::Point Point_from_PoseTwist(const Pose &pose, const Twist &twist) {
    tf::Quaternion q; tf::quaternionMsgToTF(pose.orientation, q);

    subjugator::C3Trajectory::Point res;

    res.q.head(3) = xyz2vec(pose.position);
    tf::Matrix3x3(q).getRPY(
        res.q[3],
        res.q[4],
        res.q[5]);

    res.qdot.head(3) = vec2vec(tf::Matrix3x3(q) * vec2vec(xyz2vec(twist.linear)));
    res.qdot.tail(3) = (Eigen::Matrix3d() <<
        1, sin(res.q[3]) * tan(res.q[4]),  cos(res.q[3]) * tan(res.q[4]),
        0, cos(res.q[3])                , -sin(res.q[3])                ,
        0, sin(res.q[3]) / cos(res.q[4]),  cos(res.q[3]) / cos(res.q[4])
    ).finished() * xyz2vec(twist.angular);

    return res;
}

// create special UF-Common 'PoseTwist' object from C3 'PointWithAcceleration' object
PoseTwist PoseTwist_from_PointWithAcceleration(const subjugator::C3Trajectory::PointWithAcceleration &p) {
    tf::Quaternion orient = tf::createQuaternionFromRPY(p.q[3], p.q[4], p.q[5]);

    PoseTwist res;

    res.pose.position = vec2xyz<Point>(p.q.head(3));
    quaternionTFToMsg(orient, res.pose.orientation);

    Eigen::Matrix3d worldangvel_from_eulerrates = (Eigen::Matrix3d() <<
        1,            0,              -sin(p.q[4]),
        0,  cos(p.q[3]), sin(p.q[3]) * cos(p.q[4]),
        0, -sin(p.q[3]), cos(p.q[3]) * cos(p.q[4])
    ).finished();

    res.twist.linear = vec2xyz<Vector3>(tf::Matrix3x3(orient.inverse()) * vec2vec(p.qdot.head(3)));
    res.twist.angular = vec2xyz<Vector3>(worldangvel_from_eulerrates * p.qdot.tail(3));

    res.acceleration.linear = vec2xyz<Vector3>(tf::Matrix3x3(orient.inverse()) * vec2vec(p.qdotdot.head(3)));
    res.acceleration.angular = vec2xyz<Vector3>(worldangvel_from_eulerrates * p.qdotdot.tail(3));

    return res;
}

// take C3 'Waypoint' object, translate to geometry_msgs::Pose
Pose Pose_from_Waypoint(const subjugator::C3Trajectory::Waypoint &wp) {
    Pose res;

    res.position = vec2xyz<Point>(wp.r.q.head(3));
    quaternionTFToMsg(tf::createQuaternionFromRPY(wp.r.q[3], wp.r.q[4], wp.r.q[5]),
        res.orientation);

    return res;
}

struct Node {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    tf::TransformListener tf_listener;
    kill_handling::KillListener kill_listener;

    string body_frame;
    string fixed_frame;
    subjugator::C3Trajectory::Limits limits;
    ros::Duration traj_dt;

    ros::Subscriber odom_sub;
    actionlib::SimpleActionServer<uf_common::MoveToAction> actionserver;
    	// server object used to send out "Move To" actions
    ros::Publisher trajectory_pub;
    ros::Publisher waypoint_pose_pub;
    ros::ServiceServer set_disabled_service;

    ros::Timer update_timer;

    bool disabled;
    boost::scoped_ptr<subjugator::C3Trajectory> c3trajectory;
    ros::Time c3trajectory_t;

    subjugator::C3Trajectory::Waypoint current_waypoint;
    ros::Time current_waypoint_t;

    double linear_tolerance, angular_tolerance;
    
    // if kill message issued, reset the c3 pointer
    void killed_callback() {
        c3trajectory.reset(); // boost::scoped_ptr.reset()
    }
    
    bool set_disabled(SetDisabledRequest & request, SetDisabledResponse& response) {
        disabled = request.disabled;
        if(disabled) {
            c3trajectory.reset(); // boost::scoped_ptr.reset()
        }
        return true;
    }

    Node() : // now the initialization list
        private_nh("~"),
        kill_listener(boost::bind(&Node::killed_callback, this)),
        actionserver(nh, "moveto", false),	// what is the actionserver binding to?
			// ActionServer(ros::NodeHandle, std::string name, bool auto_start)
        disabled(false) {

    	// frame parameters - see next section
        fixed_frame = uf_common::getParam<std::string>(private_nh, "fixed_frame");
        body_frame = uf_common::getParam<std::string>(private_nh, "body_frame");

        // tunable parameters - check in boat_launch/launch/controller.launch
        limits.vmin_b = uf_common::getParam<subjugator::Vector6d>(private_nh, "vmin_b");
        limits.vmax_b = uf_common::getParam<subjugator::Vector6d>(private_nh, "vmax_b");
        limits.amin_b = uf_common::getParam<subjugator::Vector6d>(private_nh, "amin_b");
        limits.amax_b = uf_common::getParam<subjugator::Vector6d>(private_nh, "amax_b");
        limits.arevoffset_b = uf_common::getParam<Eigen::Vector3d>(private_nh, "arevoffset_b");
        limits.umax_b = uf_common::getParam<subjugator::Vector6d>(private_nh, "umax_b");
        traj_dt = uf_common::getParam<ros::Duration>(private_nh, "traj_dt", ros::Duration(0.0001)); // default to Duration(.0001)

        // odom will give an estimate on position and velocity in free space
        odom_sub = nh.subscribe<Odometry>("odom", 1, boost::bind(&Node::odom_callback, this, _1));
        	// this data likely comes from gps; call call 'odom_callback()'

        waypoint_pose_pub = private_nh.advertise<PoseStamped>("waypoint", 1);
        	// PoseStamped : position and orientation (includes a Header)
        trajectory_pub = nh.advertise<PoseTwistStamped>("trajectory", 1);
        	// PoseTwistStamped : PoseStamped + linear and angular vectors of velocity

        update_timer = nh.createTimer(ros::Duration(1./50), boost::bind(&Node::timer_callback, this, _1));
        	// call 'timer_callback()' every... idk what "./" means.  result is ambiguous
        	// see line 179 at http://docs.ros.org/diamondback/api/rostime/html/duration_8h_source.html

        actionserver.start();
        
        set_disabled_service = private_nh.advertiseService
            <SetDisabledRequest, SetDisabledResponse>(
            "set_disabled", boost::bind(&Node::set_disabled, this, _1, _2));
    }

    void odom_callback(const OdometryConstPtr& odom) {
        if(c3trajectory) // boost::scoped_ptr<subjugator::C3Trajectory>
            return; // already initialized
        if(kill_listener.get_killed() || disabled)
            return; // only initialize when unkilled

        subjugator::C3Trajectory::Point current = Point_from_PoseTwist(odom->pose.pose, odom->twist.twist);
        	// function within 'node.cpp'
        current.q[3] = current.q[4] = 0; // zero roll and pitch
        current.qdot = subjugator::Vector6d::Zero(); // zero velocities

        c3trajectory.reset(new subjugator::C3Trajectory(current, limits));
        c3trajectory_t = odom->header.stamp;

        current_waypoint = current; // set desired waypoint to calculated Point from PoseTwist
        current_waypoint_t = odom->header.stamp; // timestamp of waypoint change
    }

    // timer being called every <ambiguous> amount of time
    void timer_callback(const ros::TimerEvent&) {
        if(!c3trajectory)
            return;

        ros::Time now = ros::Time::now();

        if(actionserver.isNewGoalAvailable()) {
            boost::shared_ptr<const uf_common::MoveToGoal> goal = actionserver.acceptNewGoal();
            current_waypoint = subjugator::C3Trajectory::Waypoint(
                Point_from_PoseTwist(goal->posetwist.pose, goal->posetwist.twist),
                goal->speed, !goal->uncoordinated);
            current_waypoint_t = now; // goal->header.stamp;
            this->linear_tolerance = goal->linear_tolerance;
            this->angular_tolerance = goal->angular_tolerance;
        }
        if(actionserver.isPreemptRequested()) {
            current_waypoint = c3trajectory->getCurrentPoint();
            current_waypoint.r.qdot = subjugator::Vector6d::Zero(); // zero velocities
            current_waypoint_t = now;

            // don't try to make output c3 continuous when cancelled - instead stop as quickly as possible
            c3trajectory.reset(new subjugator::C3Trajectory(current_waypoint.r, limits));
            c3trajectory_t = now;
        }

        while(c3trajectory_t + traj_dt < now) {
            c3trajectory->update(traj_dt.toSec(), current_waypoint, (c3trajectory_t - current_waypoint_t).toSec());
            c3trajectory_t += traj_dt;
        }

        PoseTwistStamped msg;
        msg.header.stamp = c3trajectory_t;
        msg.header.frame_id = fixed_frame;
        msg.posetwist = PoseTwist_from_PointWithAcceleration(c3trajectory->getCurrentPoint());
        trajectory_pub.publish(msg);

        PoseStamped posemsg;
        posemsg.header.stamp = c3trajectory_t;
        posemsg.header.frame_id = fixed_frame;
        posemsg.pose = Pose_from_Waypoint(current_waypoint);
        waypoint_pose_pub.publish(posemsg);

        if(actionserver.isActive() && c3trajectory->getCurrentPoint().is_approximately(current_waypoint.r, max(1e-3, linear_tolerance), max(1e-3, angular_tolerance)) && current_waypoint.r.qdot == subjugator::Vector6d::Zero()) {
            actionserver.setSucceeded();
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "c3_trajectory_generator");

    Node n; // see Node struct above

    ros::spin();

    return 0;
}
