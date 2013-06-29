#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include <stdio.h>
#include <stdexcept>
#include <iostream>

#include <opencv/highgui.h>

#include "Contours.h"

using namespace cv;
using namespace std;

class CornerComparator {
	private:
		Point centroid;
	public:
		CornerComparator(Point centroid) : centroid(centroid) {}
		bool operator()(Point a, Point b) {
			float angle_a = atan2(a.x-centroid.x, -(a.y-centroid.y));
			float angle_b = atan2(b.x-centroid.x, -(b.y-centroid.y));
			return angle_a < angle_b;
		}
};

Contours::Contours(const Mat &img, float minContour, float maxContour, float maxPerimeter) {
	Mat dbg_temp = img.clone();
	cv::findContours(dbg_temp,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);

	for( size_t i = 0; i < contours.size(); i++ ) {
		// only process positive contours
		int nparents = 0;
		for(int j = hierarchy[i][3]; j >= 0; j = hierarchy[j][3]) // for every parent up to root
			nparents++;
		if(nparents % 2) // if this node has an odd number of parents
			continue; // skip it

		float area_holder = fabs(contourArea(Mat(contours[i])));
		float perimeter_holder = arcLength(Mat(contours[i]), true);
		if(area_holder < minContour || area_holder > maxContour || perimeter_holder > maxPerimeter)
			continue;

		// add inner contours to "shapes" member and prepare them for box's "shapes" member if box is found
		vector<Contours::InnerContour> innerContours;
		for(int j = hierarchy[i][2]; j >= 0; j = hierarchy[j][0]) { // for every child contour
			float inner_area_holder = fabs(contourArea(Mat(contours[j])));
			float inner_perimeter_holder = arcLength(Mat(contours[j]), true);
			if(inner_area_holder < minContour || inner_area_holder > maxContour
				|| inner_perimeter_holder > maxPerimeter )
				continue;

			Point2f center_holder;float radius_holder;minEnclosingCircle(Mat(contours[j]),center_holder,radius_holder);

			if(center_holder.x == 0 || center_holder.y == 0) continue; // ???

			InnerContour innerContour;
			innerContour.perimeter = inner_perimeter_holder;
			innerContour.area = inner_area_holder;
			innerContour.centroid.x = (int)center_holder.x;
			innerContour.centroid.y = (int)center_holder.y;
			innerContour.radius = radius_holder;
			innerContour.outer_area = area_holder;
			//RotatedRect rr = fitEllipse(contours[j]);
			//innerContour.circularity = (boost::math::constants::pi<double>()*rr.size.width*rr.size.height/4)/inner_area_holder;
			innerContour.circularity = inner_area_holder/(boost::math::constants::pi<double>()*pow(radius_holder, 2));
			innerContour.contour.push_back(contours[j]);
			shapes.push_back(innerContour);
			innerContours.push_back(innerContour);
		}

		// try to find box

		// approximate contour with accuracy proportional to the contour perimeter
		vector<Point> approx;
		approxPolyDP(Mat(contours[i]), approx, perimeter_holder*0.03, true);

		// square contours should have 4 vertices after approximation and be convex.
		if(approx.size() != 4 || !isContourConvex(Mat(approx)))
			continue;

		// find the maximum cosine of the angle between joint edges
		double maxCosine = 0;
		for(int j = 2; j < 5; j++)
			maxCosine = MAX(maxCosine, fabs(angle(approx[j%4], approx[j-2], approx[j-1])));
		if(maxCosine > 0.4)
			continue;

		// if cosines of all angles are small (all angles are ~90 degree) then write quandrange
		// vertices to resultant sequence
		OuterBox outerBox;
		outerBox.corners = approx;
		outerBox.area = area_holder;
		outerBox.perimeter = perimeter_holder;
		outerBox.centroid.x = (approx[0].x + approx[1].x + approx[2].x + approx[3].x)/4;
		outerBox.centroid.y = (approx[0].y + approx[1].y + approx[2].y + approx[3].y)/4;
		outerBox.contour.push_back(contours[i]);
		outerBox.shapes = innerContours;

		sort(outerBox.corners.begin(), outerBox.corners.end(), CornerComparator(outerBox.centroid));

		double right_line = norm(outerBox.corners[1] - outerBox.corners[0]);
		double left_line = norm(outerBox.corners[3] - outerBox.corners[2]);
		outerBox.orientationError = right_line - left_line;

		double length1 = norm(outerBox.corners[1] - outerBox.corners[0]);
		double length2 = norm(outerBox.corners[3] - outerBox.corners[0]);
		if(length1 > length2)
			outerBox.orientation = 0.5*(outerBox.corners[0] + outerBox.corners[3]);
		else
			outerBox.orientation = 0.5*(outerBox.corners[0] + outerBox.corners[1]);

		Point dp = outerBox.orientation - outerBox.centroid;
		outerBox.angle = atan2(dp.y, dp.x) + CV_PI/2;

		outerBox.touches_edge = false;
		BOOST_FOREACH(const Point& p, outerBox.corners)
			if(p.x <= 1 || p.y >= img.cols-2 || p.y <= 1 || p.y >= img.rows-2)
				outerBox.touches_edge = true;
		if(!outerBox.touches_edge)
			boxes.push_back(outerBox);
	}
}

double Contours::angle( Point pt1, Point pt2, Point pt0 )
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void Contours::drawResult(Mat &img, const Scalar &color) {
	drawContours(img, contours, -1, Scalar(255, 255, 255), 1, 8, hierarchy, 0); // draw all contours

	BOOST_FOREACH(const OuterBox &box, boxes) {
		circle(img, box.centroid, 2, color, 2, 8, 0);
		drawContours(img, box.contour, 0, color, 2, 8, hierarchy, 0);
		for(size_t j=0; j < box.corners.size(); j++)
		{
			circle(img,box.corners[j],3,CV_RGB(255,255,0),-1,8);
			ostringstream os; os << j;
			putText(img,os.str().c_str(),Point(box.corners[j].x+5,box.corners[j].y+5),FONT_HERSHEY_SIMPLEX,0.3,CV_RGB(255,255,255),1);
		}
		line(img,box.centroid,box.orientation,CV_RGB(255,0,0),2,8);
	}
	BOOST_FOREACH(const InnerContour &shape, shapes) {
		circle(img, shape.centroid, 5, CV_RGB(255,255,255), 2, 8, 0);
		circle(img, shape.centroid, (int)shape.radius, CV_RGB(255, 255, 255), 1, 8);
		drawContours(img, shape.contour, 0, CV_RGB(0, 0, 50), 2, 8, hierarchy, 0);
		ostringstream os; os << "Area: " << shape.area << " " << shape.circularity;
		putText(img,os.str().c_str(),shape.centroid,FONT_HERSHEY_SIMPLEX,1,CV_RGB(255,255,255),1);
	}
}

Contours::InnerContour Contours::findLargestShape() {
	bool foundSomething = false;
	InnerContour bestShape;
	BOOST_FOREACH(const InnerContour &shape, shapes)
		if(!foundSomething || shape.area > bestShape.area) {
			foundSomething = true;
			bestShape = shape;
		}
	assert(foundSomething);
	return bestShape;
}

Contours::InnerContour Contours::findSmallestShape() {
	bool foundSomething = false;
	InnerContour bestShape;
	BOOST_FOREACH(const InnerContour &shape, shapes)
		if(!foundSomething || shape.area < bestShape.area) {
			foundSomething = true;
			bestShape = shape;
		}
	assert(foundSomething);
	return bestShape;
}

Point Contours::calcCentroidOfAllBoxes() {
	assert(boxes.size());
	Point sum(0, 0);
	BOOST_FOREACH(const OuterBox &box, boxes)
		sum += box.centroid;
	return 1./boxes.size()*sum;
}

float Contours::calcAngleOfAllBoxes() {
	assert(boxes.size());
	float sum = 0;
	BOOST_FOREACH(const OuterBox &box, boxes)
		sum += box.angle;
	return sum/boxes.size();
}
