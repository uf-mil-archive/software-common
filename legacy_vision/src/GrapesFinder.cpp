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
        Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 1);

        Mat normalized = Normalizer::normRGB(blurred);

        // call to thresholder here
        Mat thresholded;
        if(objectPath[0] == "board") {
                thresholded = Thresholder(normalized).simpleRGB(Vec3b(85, 85, 85), Vec3b(0, 128, 128));
	        erode(thresholded, thresholded, cv::Mat::ones(3,3,CV_8UC1));
        } else if(objectPath[0] == "empty_cell") {
                thresholded = Thresholder(normalized).simpleRGB(Vec3b(0, 255, 255), Vec3b(255, 0, 0));
        } else if(objectPath[0] == "peg") {
                thresholded = Thresholder(normalized).simpleRGB(Vec3b(255, 0, 0), Vec3b(0, 0, 255));
        } else {
                throw std::runtime_error("Invalid object path");
        }
        /*
        erode(thresholded, thresholded, cv::Mat::ones(3,3,CV_8UC1)); // -1
        */
        Blob blob(thresholded, 300, 1e10, 1e10, false, true);
        
        if(objectPath[0] != "board") {
            for(unsigned int i = 0; i < blob.data.size(); )
                if(blob.data[i].circularity < .8)
                        blob.data.erase(blob.data.begin()+i);
                else
                        i++;
        } else {
            Mat thresholded2 = Mat::zeros(img.image.rows, img.image.cols, CV_8UC1);
            for(unsigned int i = 0; i < blob.data.size(); i++) {
		drawContours(thresholded2, std::vector<std::vector<cv::Point> >(1, blob.data[i].contour), 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
            }
            dilate(thresholded2, thresholded2, cv::Mat::ones(15,15,CV_8UC1));
            erode(thresholded2, thresholded2, cv::Mat::ones(15,15,CV_8UC1));
            blob = Blob(thresholded2, 300, 1e10, 1e10, false, true);
        }

        Mat res = normalized.clone();
        //if(blob.data.size() > 2)
        //        blob.data.resize(2);
        blob.drawResult(res, CV_RGB(0, 255, 0));

        // Prepare results
        vector<property_tree::ptree> resultVector;
        BOOST_FOREACH(const Blob::BlobData &item, blob.data) {
                property_tree::ptree fResult;
                fResult.put_child("center", Point_to_ptree(item.rect_center, img));
                fResult.put_child("direction", Direction_to_ptree(item.rect_center, item.direction, img));
                fResult.put("direction_symmetry", 2);
        
		        Mat tempMask = Mat::zeros(img.image.rows, img.image.cols, CV_8UC1);
		        drawContours(tempMask, std::vector<std::vector<cv::Point> >(1, item.contour), 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
		        fResult.put("redness", mean(normalized, tempMask)[2]);
                resultVector.push_back(fResult);
        }

        return FinderResult(resultVector, res, thresholded);
}
