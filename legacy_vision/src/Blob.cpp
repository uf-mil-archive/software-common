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

Blob::Blob(const Mat &img, float minContour, float maxContour, float maxPerimeter, bool sortByRadius, bool allowInternal, Blob::IntrusionMode intrusionMode, bool allowEdge) {
	Mat dbg_temp = img.clone();
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(dbg_temp,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);

	int i = -1;
	BOOST_FOREACH(std::vector<cv::Point> contour, contours) { i += 1;
                // only process positive contours
                int nparents = 0;
                for(int j = hierarchy[i][3]; j >= 0; j = hierarchy[j][3]) // for every parent up to root
                        nparents++;
                if(nparents % 2) // if this node has an odd number of parents
                        continue; // skip it
                if(nparents != 0 && !allowInternal) continue;
		
		if(intrusionMode == INTRUSION_DEFAULT) {
		} else if(intrusionMode == INTRUSION_IGNORE) {
			vector<Point> convex_hull; convexHull(Mat(contour), convex_hull);
			contour = convex_hull;
		} else if(intrusionMode == INTRUSION_SELECT) {
		} else {
			assert(false);
		}

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
		
		bool touches_edge = false;
		BOOST_FOREACH(const Point& p, contour)
			if(p.x <= 1 || p.x >= img.cols-2 || p.y <= 1 || p.y >= img.rows-2)
				touches_edge = true;
		if(touches_edge && !allowEdge) continue;

		RotatedRect rr = minAreaRect(Mat(contour));
        if(rr.size.width < rr.size.height) { // force width > height
            rr.angle += 90;
            std::swap(rr.size.width, rr.size.height);
        }
        double rr_angle_rad = rr.angle / 180 * pi<double>();
        
		BlobData bdata;

        double total_inner_area_holder = 0;
        for(int j = hierarchy[i][2]; j >= 0; j = hierarchy[j][0]) { // for every child contour
                float inner_area_holder = fabs(contourArea(Mat(contours[j])));
                total_inner_area_holder += inner_area_holder;
                bdata.holes.push_back(contours[j]);
        }

        
		//circle(ioimages->res,center_holder,2,CV_RGB(0,255,255),-1,8,0);
		bdata.perimeter = (float)perimeter_holder;
		bdata.area = area_holder;
		bdata.centroid.x = (int)center_holder.x;
		bdata.centroid.y = (int)center_holder.y;
		bdata.radius = radius_holder;
                bdata.direction = radius_holder * cv::Point2f(cos(rr_angle_rad), sin(rr_angle_rad));
		bdata.circularity = convex_area_holder/(pi<double>()*pow(radius_holder, 2));
		bdata.circularity_not_hull = area_holder/(pi<double>()*pow(radius_holder, 2));
		bdata.hollowness = total_inner_area_holder / area_holder;
		bdata.contour = contour;
		bdata.rect_center = rr.center;

		bdata.aspect_ratio = rr.size.width/rr.size.height;
                bdata.is_vertical = pow(sin(rr_angle_rad), 2) > pow(cos(rr_angle_rad), 2);
        bdata.short_length = rr.size.height;
        bdata.long_length = rr.size.width;
        
		//approxPolyDP(Mat(convex_hull), bdata.approx_contour, perimeter_holder*0.03, true);

		if(intrusionMode == INTRUSION_SELECT) {
			Mat img2;
			vector<Point> convex_hull; convexHull(Mat(contour), convex_hull);
			
            Mat thresholded2 = Mat::zeros(img.rows, img.cols, CV_8UC1);
			drawContours(thresholded2, std::vector<std::vector<cv::Point> >(1, convex_hull), 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
			thresholded2 -= img;
			bdata.intrusions = thresholded2;
		}

		int parent = hierarchy[i][3];
		if(parent > 0) {
			float area_holder = fabs(contourArea(Mat(contours[parent])));
			bdata.parent_area = area_holder;
		} else {
			bdata.parent_area = -1;
		}

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
		drawContours(img, std::vector<std::vector<cv::Point> >(1, item.contour), 0, CV_RGB(255,0,0), 2, 8);
		//drawContours(img, std::vector<std::vector<cv::Point> >(1, item.approx_contour), 0, CV_RGB(0,255,0), 2, 8);
		circle(img, item.centroid, (int)item.radius,color, 2, 8, 0);
		line(img, item.centroid,
		     item.centroid + item.direction, CV_RGB(255,0,0),2,8);

		std::ostringstream os;
		os << "A " << (int)item.area << " "
		   << "R " << std::setprecision(3) << item.radius;
		putText(img, os.str().c_str(), Point(item.centroid.x-30,item.centroid.y-10), FONT_HERSHEY_DUPLEX, 1, CV_RGB(0,255,0), 1.5);
		std::ostringstream os2;
		os2 << std::setprecision(3) << item.circularity_not_hull << " " << item.hollowness << " " << item.parent_area;
		putText(img, os2.str().c_str(), Point(item.centroid.x-30,item.centroid.y+10), FONT_HERSHEY_DUPLEX, 1, CV_RGB(0,255,0), 1.5);
	}

}
