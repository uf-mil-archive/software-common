#ifndef LINE_H
#define LINE_H

#include <vector>
#include <boost/property_tree/ptree.hpp>

#include "AvgLine.h"

class Line {
	public:
		std::vector<AvgLine> avgLines;
		Line(int numberOfLinesToFind, boost::property_tree::ptree config, const cv::Mat &img);
		void drawResult(cv::Mat &img);
};

#endif
