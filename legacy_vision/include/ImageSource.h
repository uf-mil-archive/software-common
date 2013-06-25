#ifndef IMAGESOURCE_H
#define IMAGESOURCE_H

#include <opencv/cv.h>


namespace subjugator {

namespace ImageSource {

struct Image {
	Image(cv::Mat image) : image(image) {}
	cv::Mat image;
};

}

}

#endif
