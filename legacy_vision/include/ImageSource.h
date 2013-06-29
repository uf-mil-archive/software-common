#ifndef IMAGESOURCE_H
#define IMAGESOURCE_H

#include <opencv/cv.h>

#include <image_geometry/pinhole_camera_model.h>


namespace subjugator {

namespace ImageSource {

struct Image {
	Image(cv::Mat image, const sensor_msgs::CameraInfo &cam_info) : image(image) {
	    camera_model.fromCameraInfo(cam_info);
	}
	cv::Mat image;
	image_geometry::PinholeCameraModel camera_model;
};

}

}

#endif
