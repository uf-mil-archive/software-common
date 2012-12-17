#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>


namespace magnetic_hardsoft_compensation {
    class Nodelet : public nodelet::Nodelet {
        private:
            std::string frame_id;
            tf::Vector3 shift;
            tf::Quaternion correction;
            tf::Vector3 scale;
            ros::Subscriber sub;
            ros::Publisher pub;
            
            tf::Vector3 get_Vector3(ros::NodeHandle& nh, const std::string& name) {
                XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
                ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(my_list.size() == 3);
                
                tf::Vector3 res;
                for (uint32_t i = 0; i < 3; i++) {
                    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                    res[i] = static_cast<double>(my_list[i]);
                }
                return res;
            }
            
            tf::Quaternion get_Quaternion(ros::NodeHandle& nh, const std::string& name) {
                XmlRpc::XmlRpcValue my_list; ROS_ASSERT(nh.getParam(name, my_list));
                ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
                ROS_ASSERT(my_list.size() == 4);
                
                tf::Quaternion res;
                for (uint32_t i = 0; i < 4; i++) {
                    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                    res[i] = static_cast<double>(my_list[i]);
                }
                return res;
            }
            
            void handle(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
                if(msg->header.frame_id != frame_id) {
                    ROS_ERROR("msg's frame_id != configured frame_id! ignoring message");
                    return;
                }
                
                tf::Vector3 raw; tf::vector3MsgToTF(msg->vector, raw);
                
                tf::Vector3 processed = quatRotate(correction.inverse(), quatRotate(correction, raw - shift) / scale);
            
                geometry_msgs::Vector3Stamped result;
                result.header.frame_id = frame_id;
                result.header.stamp = msg->header.stamp;
                tf::vector3TFToMsg(processed, result.vector);
                
                pub.publish(result);
            }
        
        public:
            Nodelet() {}
            
            virtual void onInit() {
                ros::NodeHandle& private_nh = getPrivateNodeHandle();
                
                frame_id = "/imu"; private_nh.getParam("frame_id", frame_id);
                shift = get_Vector3(private_nh, "shift");
                correction = get_Quaternion(private_nh, "correction");
                scale = get_Vector3(private_nh, "scale");
                
                
                ros::NodeHandle& nh = getNodeHandle();
                
                sub = nh.subscribe<geometry_msgs::Vector3Stamped>("imu/mag_raw", 1000, boost::bind(&Nodelet::handle, this, _1));
                pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 10);
            }

    };

    PLUGINLIB_DECLARE_CLASS(magnetic_hardsoft_compensation, nodelet, magnetic_hardsoft_compensation::Nodelet, nodelet::Nodelet);
}
