#include <cstdio>
#include <stdexcept>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include "Blob.h"

using namespace cv;
using namespace std;

Blob::Blob(const Mat &img, float minContour, float maxContour, float maxPerimeter) {
	Mat dbg_temp = img.clone();
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(dbg_temp,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

	BOOST_FOREACH(const std::vector<cv::Point> &contour, contours) {
		float area_holder = fabs(contourArea(Mat(contour)));
		vector<Point> convex_hull; convexHull(Mat(contour), convex_hull);
		float convex_area_holder = contourArea(Mat(convex_hull));
		double perimeter_holder = arcLength(Mat(contour), true);

		if(area_holder < minContour || area_holder > maxContour
				|| perimeter_holder > maxPerimeter /*|| !isContourConvex(Mat(contour))*/)
			continue;

		cv::Point2f center_holder;
		float radius_holder;
		minEnclosingCircle(Mat(contour),center_holder,radius_holder);

		if(center_holder.x == 0 || center_holder.y == 0)
			continue; // ???

		//circle(ioimages->res,center_holder,2,CV_RGB(0,255,255),-1,8,0);
		BlobData bdata;
		bdata.perimeter = (float)perimeter_holder;
		bdata.area = area_holder;
		bdata.centroid.x = (int)center_holder.x;
		bdata.centroid.y = (int)center_holder.y;
		bdata.radius = radius_holder;
		bdata.circularity = convex_area_holder/(boost::math::constants::pi<double>()*pow(radius_holder, 2));
		bdata.contour = contour;

		RotatedRect rr = minAreaRect(Mat(contour));
		rr.angle -= 90; // so 0 degress = pointing right
		if(rr.size.width < rr.size.height) { // force width > height
			rr.angle += 90;
			std::swap(rr.size.width, rr.size.height);
		}
		double pi = boost::math::constants::pi<double>();
		double angle_rad = rr.angle * pi / 180;
		if(
			pointPolygonTest(Mat(contour), Point2f(rr.center) + Point2f(cos(angle_rad+pi), -sin(angle_rad+pi))*(rr.size.width/2), true) >
			pointPolygonTest(Mat(contour), Point2f(rr.center) + Point2f(cos(angle_rad   ), -sin(angle_rad   ))*(rr.size.width/2), true)
		)
			rr.angle += 180;
		angle_rad = rr.angle * pi / 180;
		//circle(ioimages->res, Point2f(rr.center) + Point2f(cos(angle_rad), -sin(angle_rad))*(rr.size.width/2),5,CV_RGB(255,255,255),2,8,0);
		bdata.angle = rr.angle * boost::math::constants::pi<double>() / 180;
		bdata.aspect_ratio = rr.size.width/rr.size.height;
		bdata.is_vertical = pow(sin(bdata.angle), 2) > pow(cos(bdata.angle), 2);

		data.push_back(bdata);
	}

	// sort largest area to smallest
	sort(data.begin(), data.end());
	reverse(data.begin(),data.end());
}

void Blob::drawResult(Mat &img, const Scalar &color) {
	BOOST_FOREACH(const BlobData &item, data) {
		circle(img, item.centroid, (int)item.radius,color, 2, 8, 0);
		line(img, item.centroid, item.centroid + Point(item.radius*cos(item.angle), item.radius*-sin(item.angle)), CV_RGB(255,0,0),2,8);

		std::ostringstream os; os << "Area: " << (int)item.area << " " << (int)(item.angle*180/boost::math::constants::pi<double>()) << " " << item.aspect_ratio;
		putText(img, os.str().c_str(), Point(item.centroid.x-30,item.centroid.y-10), FONT_HERSHEY_DUPLEX, 1, CV_RGB(0,0,0), 1.5);
	}

}
