#ifndef NORMALIZER_H
#define NORMALIZER_H

#include <opencv/cv.h>

class Normalizer {
	public:
		static cv::Mat normRGB(const cv::Mat &img);
};

#endif
