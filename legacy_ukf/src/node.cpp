#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <uf_common/param_helpers.h>
#include <uf_common/msg_helpers.h>
#include <depth_driver/Float64Stamped.h>

#include "NavigationComputer.h"
#include "Quaternion.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace depth_driver;
using namespace uf_common;

struct Node {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    tf::TransformListener tf_listener;
    
    string body_frame;
    string fixed_frame;
    
    message_filters::Subscriber<Imu> imu_sub;
    message_filters::Subscriber<Vector3Stamped> mag_sub;
    TimeSynchronizer<Imu, Vector3Stamped> sync;
    message_filters::Subscriber<Float64Stamped> depth_sub;
    message_filters::Subscriber<Vector3Stamped> dvl_sub;
    ros::Timer timer;
    
    ros::Publisher odometry_pub;
    tf::TransformBroadcaster tf_broadcaster;
    ros::Publisher pose_pub;
    ros::Publisher attref_pub;
    
    boost::scoped_ptr<subjugator::NavigationComputer> navComputer;
    
    Node() :
        private_nh("~"),
        imu_sub(nh, "imu/data_raw", 1),
        mag_sub(nh, "imu/mag", 1),
        sync(imu_sub, mag_sub, 10),
        depth_sub(nh, "depth", 1),
        dvl_sub(nh, "dvl", 1) {
        
        ROS_ASSERT(private_nh.getParam("fixed_frame", fixed_frame));
        ROS_ASSERT(private_nh.getParam("body_frame", body_frame));
        
        subjugator::NavigationComputer::Config navconf;
        navconf.referenceNorthVector = get_Vector3(private_nh, "referenceNorthVector");
        ROS_ASSERT(private_nh.getParam("latitudeDeg", navconf.latitudeDeg));
        navconf.dvl_sigma = get_Vector3(private_nh, "dvl_sigma");
        navconf.att_sigma = get_Vector3(private_nh, "att_sigma");
        
        navComputer.reset(new subjugator::NavigationComputer(navconf));
        
        odometry_pub = nh.advertise<Odometry>("odom", 1);
        pose_pub = private_nh.advertise<PoseStamped>("pose", 1);
        attref_pub = private_nh.advertise<PoseStamped>("attref", 1);
        
        sync.registerCallback(boost::bind(&Node::imu_callback, this, _1, _2));
        depth_sub.registerCallback(boost::bind(&Node::depth_callback, this, _1));
        dvl_sub.registerCallback(boost::bind(&Node::dvl_callback, this, _1));
    }
    
    void imu_callback(const ImuConstPtr& imu, const Vector3StampedConstPtr& mag) {
        assert(imu->header.stamp == mag->header.stamp);
        
        // XXX use frame_id
        
        subjugator::IMUInfo imuinfo;
        imuinfo.timestamp = imu->header.stamp;
        imuinfo.acceleration = xyz2vec(imu->linear_acceleration);
        imuinfo.ang_rate = xyz2vec(imu->angular_velocity);
        imuinfo.mag_field = xyz2vec(mag->vector);
        
        navComputer->UpdateIMU(imuinfo);
        
        publish();
    }
    
    void depth_callback(const Float64StampedConstPtr& depth) {
        navComputer->UpdateDepth(depth->data);
    }
    
    void dvl_callback(const Vector3StampedConstPtr& dvl) {
        subjugator::DVLVelocity dvlvelocity;
        dvlvelocity.vel = xyz2vec(dvl->vector);
        
        navComputer->UpdateDVL(dvlvelocity);
    }
    
    void publish() {
        if(!navComputer->getInitialized())
            return;
        
        subjugator::LPOSVSSInfo info;
        navComputer->GetNavInfo(info);
        
        // Emit the LPOSInfo every iteration
        Odometry msg;
        msg.header.stamp = info.timestamp;
        msg.header.frame_id = fixed_frame;
        msg.child_frame_id = body_frame;
        
        Eigen::Vector4d ENU_from_NED = subjugator::MILQuaternionOps::QuatNormalize(Eigen::Vector4d(0, 1, 1, 0));
        Eigen::Vector4d FLU_from_FRD = subjugator::MILQuaternionOps::QuatNormalize(Eigen::Vector4d(0, 1, 0, 0));
        
        Eigen::Vector3d position_ENU = subjugator::MILQuaternionOps::QuatRotate(ENU_from_NED, info.position_NED);
        msg.pose.pose.position = vec2xyz<Point>(position_ENU);
        Eigen::Vector4d orientation_ENU = subjugator::MILQuaternionOps::QuatMultiply(subjugator::MILQuaternionOps::QuatMultiply(ENU_from_NED, info.quaternion_NED_B), subjugator::MILQuaternionOps::QuatConjugate(FLU_from_FRD));
        msg.pose.pose.orientation = vec2wxyz<Quaternion>(orientation_ENU);
        
        Eigen::Vector3d velocity_BODY = subjugator::MILQuaternionOps::QuatRotate(
            subjugator::MILQuaternionOps::QuatConjugate(info.quaternion_NED_B),
            info.velocity_NED);
        msg.twist.twist.linear = vec2xyz<Vector3>(subjugator::MILQuaternionOps::QuatRotate(FLU_from_FRD, velocity_BODY));
        msg.twist.twist.angular = vec2xyz<Vector3>(subjugator::MILQuaternionOps::QuatRotate(FLU_from_FRD, info.angularRate_BODY));
        
        odometry_pub.publish(msg);
        
        PoseStamped msg3;
        msg3.header.stamp = info.timestamp;
        msg3.header.frame_id = fixed_frame;
        msg3.pose.position = vec2xyz<Point>(position_ENU);
        msg3.pose.orientation = vec2wxyz<Quaternion>(orientation_ENU);
        pose_pub.publish(msg3);
        
        PoseStamped msg2;
        msg2.header.stamp = info.timestamp;
        msg2.header.frame_id = fixed_frame;
        msg2.pose.position = vec2xyz<Point>(position_ENU);
        Eigen::Vector4d attref_ENU = subjugator::MILQuaternionOps::QuatMultiply(subjugator::MILQuaternionOps::QuatMultiply(ENU_from_NED, navComputer->getAttRef()), subjugator::MILQuaternionOps::QuatConjugate(FLU_from_FRD));
        msg2.pose.orientation = vec2wxyz<Quaternion>(attref_ENU);
        attref_pub.publish(msg2);
        
        
        tf::Transform transform; poseMsgToTF(msg.pose.pose, transform);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, msg.header.stamp, msg.header.frame_id, msg.child_frame_id));
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "legacy_ukf");
    
    Node n;
    
    ros::spin();
    
    return 0;
}
