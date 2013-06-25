#ifndef THRESHOLDER_H
#define THRESHOLDER_H

#include <opencv/cv.h>

class Thresholder {
	public:
		Thresholder(const cv::Mat &img);
		cv::Mat orange();
		cv::Mat red();
		cv::Mat shooterRed();
		cv::Mat green();
		cv::Mat yellow();
		cv::Mat blue();
		cv::Mat black();
	private:
		std::vector<cv::Mat> channelsRGB;
		std::vector<cv::Mat> channelsLAB;
		std::vector<cv::Mat> channelsHSV;
};

#endif
