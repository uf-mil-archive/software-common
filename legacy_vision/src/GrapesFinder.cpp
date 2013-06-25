#include <boost/foreach.hpp>

#include <stdio.h>
#include <iostream>

#include "Blob.h"
#include "Contours.h"
#include "Normalizer.h"
#include "Thresholder.h"

#include "GrapesFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

IFinder::FinderResult GrapesFinder::find(const subjugator::ImageSource::Image &img) {
	// call to normalizer here
	const Mat normalized = img.image;

	// blur the image to remove noise
	//Mat blurred; GaussianBlur(normalized, blurred, Size(5,5), 10, 15, BORDER_DEFAULT);

	Thresholder thresholder(normalized);

	Mat yellow = thresholder.yellow();
	dilate(yellow, yellow, cv::Mat::ones(5,5,CV_8UC1));
	erode(yellow, yellow, cv::Mat::ones(9,9,CV_8UC1));

	Contours contours(yellow, 1000, 7000000, 1500000);

	if(objectPath[0] == "board") {
		vector<property_tree::ptree> resultVector;
		if(contours.boxes.size()) {
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(contours.boxes[0].centroid, img.image.size()));
			fResult.put("scale", contours.boxes[0].area);
			fResult.put("angle", contours.boxes[0].orientationError);
			resultVector.push_back(fResult);
		}

		Mat res = img.image.clone();
		contours.drawResult(res, CV_RGB(128, 255, 128));

		return FinderResult(resultVector, res, yellow);
	} else if(objectPath[0] == "grape") {
		// mask gaps within yellow
		Mat tempMask = Mat::zeros(img.image.size(), CV_8UC1);
		BOOST_FOREACH(const Contours::InnerContour &shape, contours.shapes)
			drawContours(tempMask, shape.contour, 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
		dilate(tempMask, tempMask, cv::Mat::ones(5,5,CV_8UC1));
		Mat red = thresholder.shooterRed();
		bitwise_and(red, tempMask, red); // use mask to only find red areas within holes in yellow
		erode(red, red, cv::Mat::ones(5,5,CV_8UC1));
		dilate(red, red, cv::Mat::ones(5,5,CV_8UC1));
		//erode(red, red, cv::Mat::ones(5,5,CV_8UC1));
		Blob blob(red, 15, 1000000, 1000000);

		vector<property_tree::ptree> resultVector;
		BOOST_FOREACH(const Blob::BlobData &b, blob.data) {
			// Prepare results
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(b.centroid, img.image.size()));
			fResult.put("scale", b.radius);
			resultVector.push_back(fResult);
		}

		Mat res = img.image.clone();
		blob.drawResult(res, CV_RGB(255, 255, 255));

		return FinderResult(resultVector, res, yellow/2 | red);
	} else { assert(objectPath[0] == "grape_close");
		Mat red = thresholder.shooterRed();
		erode(red, red, cv::Mat::ones(5,5,CV_8UC1));
		dilate(red, red, cv::Mat::ones(7,7,CV_8UC1));
		//erode(red, red, cv::Mat::ones(9,9,CV_8UC1));

		Blob blob(red, 100, 10000, 1000000);

		// Prepare results
		vector<property_tree::ptree> resultVector;
		BOOST_FOREACH(const Blob::BlobData &b, blob.data) {
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(b.centroid, img.image.size()));
			fResult.put("scale", b.radius);
			resultVector.push_back(fResult);
		}

		// Draw result
		Mat res = img.image.clone();
		blob.drawResult(res, CV_RGB(255, 255, 255));

		return FinderResult(resultVector, res, red);
	}
}
