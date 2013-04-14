#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <uf_common/param_helpers.h>
#include <uf_common/msg_helpers.h>


namespace magnetic_hardsoft_compensation {
    class Nodelet : public nodelet::Nodelet {
        private:
            std::string frame_id;
            Eigen::Matrix3d scale_inverse;
            Eigen::Vector3d shift;
            ros::Subscriber sub;
            ros::Publisher pub;
            
            void handle(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
                if(msg->header.frame_id != frame_id) {
                    ROS_ERROR("msg's frame_id != configured frame_id! ignoring message");
                    return;
                }
                
                Eigen::Vector3d raw = uf_common::xyz2vec(msg->vector);
                
                Eigen::Vector3d processed = scale_inverse * (raw - shift);
                
                geometry_msgs::Vector3Stamped result;
                result.header = msg->header;
                result.vector = uf_common::vec2xyz<geometry_msgs::Vector3>(processed);
                
                pub.publish(result);
            }
        
        public:
            Nodelet() {}
            
            virtual void onInit() {
                ros::NodeHandle& private_nh = getPrivateNodeHandle();
                
                ROS_ASSERT(private_nh.getParam("frame_id", frame_id));
                scale_inverse = uf_common::get_Matrix(private_nh, "scale").inverse();
                shift = uf_common::get_Vector(private_nh, "shift");
                
                
                ros::NodeHandle& nh = getNodeHandle();
                
                sub = nh.subscribe<geometry_msgs::Vector3Stamped>("imu/mag_raw", 1000, boost::bind(&Nodelet::handle, this, _1));
                pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 10);
            }

    };

    PLUGINLIB_DECLARE_CLASS(magnetic_hardsoft_compensation, nodelet, magnetic_hardsoft_compensation::Nodelet, nodelet::Nodelet);
}
