#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stdint.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_img_pub");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  image_transport::Publisher image_pub;
  
  image_pub = it.advertise("test_image", 10);

  ros::Rate loop_rate(1);
  
  cv::Mat img(1024, 1280, CV_8UC3);
  
  sensor_msgs::CameraInfo fake_camera_info;

  uint32_t count = 0;
  int value = 0;
  std::stringstream ss;

  while (ros::ok())
  {
    value = count%256;
    img = cv::Scalar(value,value,value);
   
    ss.str("");
    ss << "Blank image with base value " << value << ".";
    
    ROS_INFO(ss.str().c_str());

    fake_camera_info.header.seq = count;
    fake_camera_info.header.frame_id = ss.str();
    fake_camera_info.header.stamp = ros::Time::now();
    
    image_pub.publish(cv_bridge::CvImage(fake_camera_info.header, sensor_msgs::image_encodings::BGR8, img).toImageMsg());
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
