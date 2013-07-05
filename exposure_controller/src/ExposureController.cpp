#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include "exposure_controller/metered_exposure.h"

namespace enc = sensor_msgs::image_encodings;

const float pi = 3.14159;

class ExposureController
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher pub_;
  std::stringstream outputFile_; 
  std::stringstream commandVar_; 
  boost::filesystem::ofstream fileStream_; 
  float exp_tm_ms_;
  int sysret_;
  
public:
  ExposureController()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/forward_camera/image", 1, &ExposureController::imageCb, this);
    pub_ = nh_.advertise<exposure_controller::metered_exposure>("test_metered_exposure",600);
    outputFile_ << "exp_cntrl.log";
    exp_tm_ms_ = 10.0;
    sysret_ = system("rosrun dynamic_reconfigure dynparam set /ueye_manager zoom 4");
    sysret_ += system("rosrun dynamic_reconfigure dynparam set /ueye_manager auto_exposure false");
    commandVar_ << "rosrun dynamic_reconfigure dynparam set /ueye_manager exposure_time " << exp_tm_ms_;
    sysret_ += system(commandVar_.str().c_str());
    sysret_ += system("rosrun dynamic_reconfigure dynparam set /ueye_manager hardware_gamma false");
    sysret_ += system("rosrun dynamic_reconfigure dynparam set /ueye_manager gain_boost false");
    sysret_ += system("rosrun dynamic_reconfigure dynparam set /ueye_manager auto_gain false");
    sysret_ += system("rosrun dynamic_reconfigure dynparam set /ueye_manager frame_rate 10");
 
    if(sysret_==0)
        ROS_INFO("Camera successfully initialized!");
    else
        ROS_INFO("Error during camera initialization. Camera may be unresponsive!");
 
    try{
        fileStream_.open(outputFile_.str().c_str());
    }
    catch(const boost::filesystem::filesystem_error& ex){
        ROS_DEBUG("Caught File System Error: %s", ex.what());
        exit(1);
    }
  }

  ~ExposureController()
  {
     fileStream_.close(); 
  }
  
 
  float weighteds(cv::Mat &img)
  {
    int rows = img.rows;
    int cols = img.cols;
    float weights[] = {255/128.0, 255/192.0, 255/224.0, 255/240.0, 255/248.0};
    float weight = 0;
    float areas[5];
    float area = 0;
    
    int i;
    for(i=0;i<5;i++){
        areas[i] = (i+1)*rows*cols*5/100;
    }
    
    int r;
    int c;
    float sum = 0.0;
    int count = 0;
    float cen_r = (rows+1)/2.0;
    float cen_c = (cols+1)/2.0;
    for(r=0;r<rows;r++)
        for(c=0;c<cols;c++){
            weight = 0;
            area = pi*(pow((r-cen_r),2.0) + pow((c-cen_c),2.0));
            if( area < areas[4] ){
                weight = weights[4];
                if( area < areas[3] ){
                    weight = weights[3];
                    if( area < areas[2] ){
                        weight = weights[2];
                        if( area < areas[1] ){
                            weight = weights[1];
                            if( area < areas[0] ){
                                weight = weights[0];
                            }
                        }
                    }
                }
            }
            
            if(weight > 0.001){
                ++count;
                sum += weight*(img.at<cv::Vec3b>(r,c)[0]);
            }
        }
    return sum/count;
  } 
    
  float averageLab(cv::Mat &img)
  {
    int rows = img.rows;
    int cols = img.cols;

    int r;
    int c;
    double sum = 0.0;
    int count = 0;
    for(r=0;r<rows;r++)
        for(c=0;c<cols;c++){
            ++count;
            sum += img.at<cv::Vec3b>(r,c)[0];
        }
    return sum*100.0/(255.0*count);
  }
  
  void logdata(exposure_controller::metered_exposure expmsg)
  {
    try{
        fileStream_ << expmsg.exp_tms << ", ";
        fileStream_ << expmsg.lab_l << ", ";
        fileStream_ << expmsg.exp2l << std::endl;
    }
    catch(const boost::filesystem::filesystem_error& ex){
        ROS_DEBUG("Caught File System Error: %s", ex.what());
        exit(1);
    } 
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cvimage;
    try{
      cvimage = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat img;
    cv::Mat imgLab;

    img = cvimage->image;
    cv::cvtColor(img, imgLab, CV_RGB2Lab);

    
    exposure_controller::metered_exposure expmsg;
    
    expmsg.header = cvimage->header;
    expmsg.exp2l = weighteds(imgLab);
    expmsg.exp_tms = exp_tm_ms_;    
    expmsg.lab_l = averageLab(imgLab);
    
    exp_tm_ms_ = exp_tm_ms_ * 1.1;
    if(exp_tm_ms_ > 40) exp_tm_ms_ = 1;
    commandVar_.str("");
    commandVar_ << "rosrun dynamic_reconfigure dynparam set /ueye_manager exposure_time " << exp_tm_ms_;
    sysret_ += system(commandVar_.str().c_str());
    
    pub_.publish(expmsg);  
    logdata(expmsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exposure_controller");
  ExposureController ec;
  ros::spin();
  return 0;
}
