#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include "Line.h"
#include <cstdio>
#include <cmath>

using namespace cv;
using namespace std;
using namespace boost;

static double DAngleDiff(double a, double b)
{
    static double Pi = boost::math::constants::pi<double>();
    static double TwoPi = 2*Pi;
    
    double res = b-a;
    while(res < -1*Pi) res += TwoPi;
    while(res > Pi) res-= TwoPi;
    
    return res;
}


Line::Line(int numberOfLinesToFind, property_tree::ptree config, const cv::Mat &img) {
	double minAngleDiff = config.get<float>("minAngleDiff")*3.14159/180.0;
	Mat edgeImage = img.clone();
	Canny(img, edgeImage, config.get_child("Canny").get<int>("thresh1"), config.get_child("Canny").get<int>("thresh2"), config.get_child("Canny").get<int>("apertureSize") );
	std::vector<Vec4i> cvlines;HoughLinesP(edgeImage, cvlines, config.get_child("Hough").get<double>("rho"), config.get_child("Hough").get<double>("theta"), config.get_child("Hough").get<double>("thresh"), config.get_child("Hough").get<int>("minLineLength"), config.get_child("Hough").get<int>("minLineGap") );

	vector<vector<AvgLine::Line> > lines;
	BOOST_FOREACH(const Vec4i &cvline, cvlines) {
		AvgLine::Line line(Point2f(cvline[0], cvline[1]), Point2f(cvline[2], cvline[3]));
		double angle = AvgLine(vector<AvgLine::Line>(1, line)).angle;

		if(numberOfLinesToFind == 1) { // case when looking for a single line (i.e. tube)
			if(lines.size() == 0)
				lines.push_back(vector<AvgLine::Line>());

			lines[0].push_back(line);

		} else if(numberOfLinesToFind == 2) { // case when looking for two lines (i.e. pipes)
			if(lines.size() == 0) {
				// if a new angle comes in and the first average is unpopulated, save it as the first average
				lines.push_back(vector<AvgLine::Line>());
				lines[0].push_back(line);

			} else if(lines.size() == 1 && abs(DAngleDiff(AvgLine(lines[0]).angle, angle)) >= minAngleDiff) {
				// if a new angle comes in and the first average is populated and the second average is open
				// and the new angle is far from the first average, save it as the second average
				lines.push_back(vector<AvgLine::Line>());
				lines[1].push_back(line);

			} else if(abs(DAngleDiff(AvgLine(lines[0]).angle, angle)) < minAngleDiff) {
				// if a new angle comes in and both averages are populated, find which average it is closest to,
				// then call the update average helper
				lines[0].push_back(line);

			} else if(lines.size() == 2 && abs(DAngleDiff(AvgLine(lines[1]).angle, angle)) < minAngleDiff) {
				lines[1].push_back(line);
			}
		}
	}

	BOOST_FOREACH(const vector<AvgLine::Line> &line, lines)
		avgLines.push_back(AvgLine(line));
}

void Line::drawResult(Mat &img) {
	BOOST_FOREACH(const AvgLine &avgline, avgLines) {
		Point2f halflength = (avgline.length/2)*Point2f(cos(avgline.angle), sin(avgline.angle));
		line(img, avgline.centroid+halflength, avgline.centroid-halflength, Scalar(0, 255, 0), 3, 8);
		circle(img, avgline.centroid, 3, Scalar(0, 150, 255), 2);
		ostringstream os; os << "Angle: " << avgline.angle*180/3.1415;
		putText(img, os.str().c_str(), avgline.centroid, FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255, 0, 0), 1);
	}
}
