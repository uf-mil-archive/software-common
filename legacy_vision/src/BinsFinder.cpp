#include <boost/foreach.hpp>

#include <stdio.h>
#include <sstream>

#include <opencv/highgui.h>

#include "Contours.h"
#include "Normalizer.h"
#include "Thresholder.h"

#include "BinsFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

IFinder::FinderResult BinsFinder::find(const subjugator::ImageSource::Image &img) {
	Mat blurred; GaussianBlur(img.image, blurred, Size(0,0), 1.3);

	Thresholder thresholder(blurred);

	vector<property_tree::ptree> resultVector;
	Mat dbg = thresholder.black();

	Contours contours(dbg, config.get<float>("minContour"), config.get<float>("maxContour"), config.get<float>("maxPerimeter"));

	// Draw result
	Mat res = img.image.clone();
	contours.drawResult(res, CV_RGB(255, 255, 255));

	if(objectPath[0] == "all") {
		Point centroidOfBoxes = contours.calcCentroidOfAllBoxes();
		circle(res,centroidOfBoxes, 5, CV_RGB(255,140,0), -1,8);
		if(contours.boxes.size() && !(contours.boxes.size() == 1 && contours.boxes[0].touches_edge)) {
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(centroidOfBoxes, img.image.size()));
			fResult.put("number_of_boxes", contours.boxes.size());
			// Scale returns the number of boxes that are currently being found.
			// The idea is to align to centroid until 4 boxes are found.
			fResult.put("angle", contours.calcAngleOfAllBoxes());
			resultVector.push_back(fResult);
		}
	} else { assert(objectPath[0] == "single");
		BOOST_FOREACH(const Contours::OuterBox &box, contours.boxes) {
			bool touches_edge = false;
			BOOST_FOREACH(const Point& p, box.corners)
				if(p.x <= 1 || p.x >= img.image.cols-2 || p.y <= 1 || p.y >= img.image.rows-2)
					touches_edge = true;
			if(touches_edge)
					continue;

			Point2f src[4];
			for(unsigned int n = 0; n < box.corners.size(); n++)
				src[n] = Point2f(box.corners[n].x, box.corners[n].y);
			if(!(norm(src[1] - src[0]) > norm(src[3] - src[0]))) // make sure long edge matches long edge so image isn't squished
				for(unsigned int n = 0; n < box.corners.size(); n++)
					src[n] = Point2f(box.corners[(n+1)%4].x, box.corners[(n+1)%4].y);

			Point2f dst[4];
			int crop = 5;
			dst[0] = Point2f(-2*crop, -crop);
			dst[1] = Point2f(300+2*crop, -crop);
			dst[2] = Point2f(300+2*crop, 150+crop);
			dst[3] = Point2f(-2*crop, 150+crop);

			Mat t = getPerspectiveTransform(src, dst);
			Mat bin;warpPerspective(img.image, bin, t, Size(300, 150));
			
			//imshow("before", bin);
			std::vector<Mat> channels(bin.channels());
			split(bin,channels);
			// NORMALIZED RGB
			for(int i = 0; i < bin.rows; i++) {
				for(int j = 0; j < bin.cols; j++) {
					Vec3b rgb_vec = bin.at<Vec3b>(i,j);
					double sum = (double)(rgb_vec[0]+rgb_vec[1]+rgb_vec[2]+0.001);

					for(int k=0; k < bin.channels(); k++)
						bin.at<Vec3b>(i,j)[k] = (double)bin.at<Vec3b>(i,j)[k] / sum * 255;
				}
			}
			vector<Mat> vBGR; split(bin, vBGR);
			//normalize(vBGR[2],vBGR[2],0,255,NORM_MINMAX);
			Mat red; adaptiveThreshold(vBGR[2],red,255,0,THRESH_BINARY,251,-5);
			erode(red,red,cv::Mat::ones(3,3,CV_8UC1));
			//dilate(red,red,cv::Mat::ones(5,5,CV_8UC1));
			//erode(red,red,cv::Mat::ones(3,3,CV_8UC1));

			Mat redness_dbg; cvtColor(red, redness_dbg, CV_GRAY2BGR);
			warpPerspective(redness_dbg, res, t, img.image.size(), WARP_INVERSE_MAP, BORDER_TRANSPARENT);

			std::vector<std::vector<cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy; // hierarchy holder for the contour tree
			findContours(red, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
			int negative_contours = 0;
			for(unsigned int i=0; i < contours.size(); i++)
				if(hierarchy[i][3] >= 0) // has parent
					negative_contours += 1;
			Mat r = red;
			float diagonal1 = mean(Mat(r, Range(0, r.rows/2), Range(0, r.cols/2)))[0] + mean(Mat(r, Range(r.rows/2, r.rows), Range(r.cols/2, r.cols)))[0];
			float diagonal2 = mean(Mat(r, Range(0, r.rows/2), Range(r.cols/2, r.cols)))[0] + mean(Mat(r, Range(r.rows/2, r.rows), Range(0, r.cols/2)))[0];

			string best;
			if(negative_contours >= 20) best = "net";
			else if(diagonal1/diagonal2 > 2) best = "sword";
			else if(diagonal2/diagonal1 > 2) best = "trident";
			else best = "shield";
		
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(box.centroid, img.image.size()));
			fResult.put("angle", box.angle);
			fResult.put("scale", box.area);
			fResult.put("diagonal", diagonal1/diagonal2);
			fResult.put("holes_count", negative_contours);
			fResult.put("item", best);
			putText(res,best.c_str(),box.centroid,FONT_HERSHEY_SIMPLEX,1,CV_RGB(0,0,255),3);
			resultVector.push_back(fResult);
		}
	}
	return FinderResult(resultVector, res, dbg);
}
