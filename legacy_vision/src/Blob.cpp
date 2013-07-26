#include <cstdio>
#include <stdexcept>

#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include "Blob.h"

using namespace cv;
using namespace std;

using boost::math::constants::pi;

bool radius_comparator(const Blob::BlobData &blob1, const Blob::BlobData &blob2) {
    return blob1.radius < blob2.radius;
}

Blob::Blob(const Mat &img, float minContour, float maxContour, float maxPerimeter, bool sortByRadius) {
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

		RotatedRect rr = minAreaRect(Mat(contour));
        if(rr.size.width < rr.size.height) { // force width > height
            rr.angle += 90;
            std::swap(rr.size.width, rr.size.height);
        }
        double rr_angle_rad = rr.angle / 180 * pi<double>();
        
		//circle(ioimages->res,center_holder,2,CV_RGB(0,255,255),-1,8,0);
		BlobData bdata;
		bdata.perimeter = (float)perimeter_holder;
		bdata.area = area_holder;
		bdata.centroid.x = (int)center_holder.x;
		bdata.centroid.y = (int)center_holder.y;
		bdata.radius = radius_holder;
                bdata.direction = cv::Point2f(cos(rr_angle_rad), sin(rr_angle_rad));
		bdata.circularity = convex_area_holder/(pi<double>()*pow(radius_holder, 2));
		bdata.contour = contour;
		bdata.rect_center = rr.center;

		bdata.aspect_ratio = rr.size.width/rr.size.height;
                bdata.is_vertical = pow(sin(rr_angle_rad), 2) > pow(cos(rr_angle_rad), 2);
        bdata.short_length = rr.size.height;
        bdata.long_length = rr.size.width;

		data.push_back(bdata);
	}

	// sort largest area to smallest
	sort(data.begin(), data.end());
	if(sortByRadius) {
	    sort(data.begin(), data.end(), radius_comparator);
    }
	reverse(data.begin(),data.end());
}

void Blob::drawResult(Mat &img, const Scalar &color) {
	BOOST_FOREACH(const BlobData &item, data) {
		circle(img, item.centroid, (int)item.radius,color, 2, 8, 0);
		line(img, item.centroid,
		     item.centroid + item.direction*item.radius, CV_RGB(255,0,0),2,8);

		std::ostringstream os;
		os << "A " << (int)item.area << " "
		   << "R " << item.radius;
		putText(img, os.str().c_str(), Point(item.centroid.x-30,item.centroid.y-10), FONT_HERSHEY_DUPLEX, 1, CV_RGB(0,0,0), 1.5);
		std::ostringstream os2;
		os2 << item.circularity;
		putText(img, os2.str().c_str(), Point(item.centroid.x-30,item.centroid.y+10), FONT_HERSHEY_DUPLEX, 1, CV_RGB(0,0,0), 1.5);
	}

}
