#ifndef AVG_LINE_H
#define AVG_LINE_H

#include <opencv/cv.h>

class AvgLine {
	public:
		cv::Point2f centroid;
		double length;
		double angle;

		typedef std::pair<cv::Point2f, cv::Point2f> Line;
		AvgLine(std::vector<Line> lines);
};

#endif
