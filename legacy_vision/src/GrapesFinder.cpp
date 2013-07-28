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

static Vec3b red_ref(36,23,41);
static Vec3b yellow_ref(61,196,70);
static Vec3b blue_ref(242,157,17);
//       R  G  B
// red - 41,23,36
// yel - 70,196,61
// blu - 17,157,242
static Vec3b vec3b_from_ptree(property_tree::ptree pt) {
    return Vec3b(
        pt.get<int>("b"),
        pt.get<int>("g"),
        pt.get<int>("r")
    );
}

IFinder::FinderResult GrapesFinder::find(const subjugator::ImageSource::Image &img) {
	// blur the image to remove noise
        Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 5, 5);
	// call to normalizer here
	const Mat normalized = img.image;


	Thresholder thresholder(normalized);

	Mat yellow = thresholder.simpleRGB(
	    vec3b_from_ptree(config.get_child("blue_ref")),
	    vec3b_from_ptree(config.get_child("yellow_ref")), 50, 0.5);
    dilate(yellow, yellow, cv::Mat::ones(11,11,CV_8UC1));
	erode(yellow, yellow, cv::Mat::ones(17,17,CV_8UC1));

	Contours contours(yellow, 1000, 7000000, 1500000);

	if(objectPath[0] == "board") {
		vector<property_tree::ptree> resultVector;
		if(contours.boxes.size()) {
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(contours.boxes[0].centroid, img));
			fResult.put("scale", contours.boxes[0].area);
			fResult.put("angle", contours.boxes[0].orientationError);
			resultVector.push_back(fResult);
		}

		Mat res = img.image.clone();
		contours.drawResult(res, CV_RGB(128, 255, 128));

		return FinderResult(resultVector, res, yellow);
	} else if(objectPath[0] == "grape") {
	    /*
		// mask gaps within yellow
		Mat tempMask = Mat::zeros(img.image.size(), CV_8UC1);
		BOOST_FOREACH(const Contours::InnerContour &shape, contours.shapes)
			drawContours(tempMask, shape.contour, 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
		dilate(tempMask, tempMask, cv::Mat::ones(5,5,CV_8UC1));
		Mat red = thresholder.shooterRed();
		bitwise_and(red, tempMask, red); // use mask to only find red areas within holes in yellow
		erode(red, red, cv::Mat::ones(5,5,CV_8UC1));
		dilate(red, red, cv::Mat::ones(5,5,CV_8UC1)); */
		Mat red = thresholder.forrest(yellow_ref, red_ref, 10, 0.8);
		erode(red, red, cv::Mat::ones(3,3,CV_8UC1));
		dilate(red, red, cv::Mat::ones(17,17,CV_8UC1));
		erode(red, red, cv::Mat::ones(3,3,CV_8UC1));
		//erode(red, red, cv::Mat::ones(5,5,CV_8UC1));
		Blob blob(red, 1000, 1000000, 1000000);
		
		
	    for(unsigned int i = 0; i < blob.data.size(); )
		    if(blob.data[i].circularity < .5)
			    blob.data.erase(blob.data.begin()+i);
		    else
			    i++;

		vector<property_tree::ptree> resultVector;
		BOOST_FOREACH(const Blob::BlobData &b, blob.data) {
			// Prepare results
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(b.centroid, img));
			fResult.put("scale", pow(b.short_length, 2));
			resultVector.push_back(fResult);
		}

		Mat res = img.image.clone();
		blob.drawResult(res, CV_RGB(255, 255, 255));

		return FinderResult(resultVector, res, yellow/2 | red);
	} else { assert(objectPath[0] == "lever");
	    /*
		// mask gaps within yellow
		Mat tempMask = Mat::zeros(img.image.size(), CV_8UC1);
		BOOST_FOREACH(const Contours::InnerContour &shape, contours.shapes)
			drawContours(tempMask, shape.contour, 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
		dilate(tempMask, tempMask, cv::Mat::ones(5,5,CV_8UC1));
		Mat red = thresholder.shooterRed();
		bitwise_and(red, tempMask, red); // use mask to only find red areas within holes in yellow
		erode(red, red, cv::Mat::ones(5,5,CV_8UC1));
		dilate(red, red, cv::Mat::ones(5,5,CV_8UC1)); */
                Mat normalized = Normalizer::normRGB(blurred);
		Mat red = Thresholder(normalized).red();
		//erode(red, red, cv::Mat::ones(3,3,CV_8UC1));
		//dilate(red, red, cv::Mat::ones(17,17,CV_8UC1));
		//erode(red, red, cv::Mat::ones(3,3,CV_8UC1));
		//erode(red, red, cv::Mat::ones(5,5,CV_8UC1));
		Blob blob(red, 400, 1000000, 1000000);
		
		Mat res = normalized.clone();
		blob.drawResult(res, CV_RGB(255, 0, 0));
		
	    for(unsigned int i = 0; i < blob.data.size(); )
		    if(blob.data[i].circularity < .35 || blob.data[i].radius < 10 || blob.data[i].radius > 60)
			    blob.data.erase(blob.data.begin()+i);
		    else
			    i++;


		blob.drawResult(res, CV_RGB(0, 0, 255));

                if(blob.data.size()) blob.data.resize(1);
		blob.drawResult(res, CV_RGB(0, 255, 0));

		vector<property_tree::ptree> resultVector;
		BOOST_FOREACH(const Blob::BlobData &b, blob.data) {
			// Prepare results
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(b.centroid, img));
			fResult.put("scale", pow(b.short_length, 2));
			resultVector.push_back(fResult);
		}

		return FinderResult(resultVector, res, yellow/2 | red);
	}
}
