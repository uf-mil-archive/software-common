#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include <uf_common/PoseTwistStamped.h>
#include <uf_common/msg_helpers.h>
#include <kill_handling/Kill.h>
#include <kill_handling/listener.h>

#include "c3_trajectory_generator/MoveToAction.h"
#include "C3Trajectory.h"

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace uf_common;


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

PoseTwist PoseTwist_from_Point(const subjugator::C3Trajectory::Point p) {
    tf::Quaternion orient = tf::createQuaternionFromRPY(p.q[3], p.q[4], p.q[5]);
    
    PoseTwist res;
    
    res.pose.position = vec2xyz<Point>(p.q.head(3));
    quaternionTFToMsg(orient, res.pose.orientation);
    
    res.twist.linear = vec2xyz<Vector3>(tf::Matrix3x3(orient.inverse()) * vec2vec(p.qdot.head(3)));
    res.twist.angular = vec2xyz<Vector3>((Eigen::Matrix3d() <<
        1,            0,              -sin(p.q[4]),
        0,  cos(p.q[3]), sin(p.q[3]) * cos(p.q[4]),
        0, -sin(p.q[3]), cos(p.q[3]) * cos(p.q[4])
    ).finished() * p.qdot.tail(3));
    
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
    actionlib::SimpleActionServer<c3_trajectory_generator::MoveToAction> actionserver;
    ros::Publisher trajectory_pub;
    
    ros::Timer update_timer;
    
    boost::scoped_ptr<subjugator::C3Trajectory> c3trajectory;
    ros::Time c3trajectory_t;
    
    subjugator::C3Trajectory::Point current_waypoint;
    ros::Time current_waypoint_t;
    
    void killed_callback() {
        c3trajectory.reset();
    }
    
    Node() :
        private_nh("~"),
        kill_listener(boost::bind(&Node::killed_callback, this)),
        actionserver(nh, "moveto", false) {
        
        limits.vmin_b << -.2, -.5, -.4, -.75, -.5, -1;
        limits.vmax_b << .75,  .5,  .4,  .75,  .5,  1;
        limits.amin_b << -.1,-.05, -.1, -1.5, -.2,-.15;
        limits.amax_b << .18, .05,.125,  1.5,  .2, .15;
        limits.arevoffset_b << .05, 0, 0;
        limits.umax_b << .25, .25, .25, .1, .1, .1;
        traj_dt = ros::Duration(0.0001);
        
        /*
        ROS_ASSERT(private_nh.getParam("fixed_frame", fixed_frame));
        ROS_ASSERT(private_nh.getParam("body_frame", body_frame));
        
        subjugator::NavigationComputer::Config navconf;
        navconf.referenceNorthVector = get_Vector3(private_nh, "referenceNorthVector");
        ROS_ASSERT(private_nh.getParam("latitudeDeg", navconf.latitudeDeg));
        navconf.dvl_sigma = get_Vector3(private_nh, "dvl_sigma");
        navconf.att_sigma = get_Vector3(private_nh, "att_sigma");
        */
        
        odom_sub = nh.subscribe<Odometry>("odom", 1, boost::bind(&Node::odom_callback, this, _1));
        
        trajectory_pub = nh.advertise<PoseTwistStamped>("trajectory", 1);
        
        update_timer = nh.createTimer(ros::Duration(1./50), boost::bind(&Node::timer_callback, this, _1));
        
        actionserver.start();
    }
    
    void odom_callback(const OdometryConstPtr& odom) {
        if(c3trajectory)
            return; // already initialized
        if(kill_listener.get_killed())
            return; // only initialize when unkilled
        
        subjugator::C3Trajectory::Point current = Point_from_PoseTwist(odom->pose.pose, odom->twist.twist);
        
        c3trajectory.reset(new subjugator::C3Trajectory(current, limits));
        c3trajectory_t = odom->header.stamp;
        
        current_waypoint = current;
        current_waypoint.q[3] = current_waypoint.q[4] = 0; // zero roll and pitch
        current_waypoint.qdot = subjugator::Vector6d::Zero(); // zero velocities
        current_waypoint_t = odom->header.stamp;
    }
    
    void timer_callback(const ros::TimerEvent&) {
        if(!c3trajectory)
            return;
        
        ros::Time now = ros::Time::now();
        
        if(actionserver.isNewGoalAvailable()) {
            boost::shared_ptr<const c3_trajectory_generator::MoveToGoal> goal = actionserver.acceptNewGoal();
            current_waypoint = Point_from_PoseTwist(goal->pose, Twist());
            current_waypoint_t = now; // goal->pose.stamp;
        }
        if(actionserver.isPreemptRequested()) {
            current_waypoint = c3trajectory->getCurrentPoint();
            current_waypoint.qdot = subjugator::Vector6d::Zero(); // zero velocities
            current_waypoint_t = now;
        }
        
        while(c3trajectory_t + traj_dt < now) {
            c3trajectory->update(traj_dt.toSec(), current_waypoint, (c3trajectory_t - current_waypoint_t).toSec());
            c3trajectory_t += traj_dt;
        }
	    
        PoseTwistStamped msg;
        msg.header.stamp = c3trajectory_t;
        msg.header.frame_id = "/map"; // XXX
        msg.posetwist = PoseTwist_from_Point(c3trajectory->getCurrentPoint());
	    
	    trajectory_pub.publish(msg);
	    
	    if(actionserver.isActive() && c3trajectory->getCurrentPoint().is_approximately(current_waypoint)) {
	        actionserver.setSucceeded();
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "c3_trajectory_generator");
    
    Node n;
    
    ros::spin();
    
    return 0;
}
