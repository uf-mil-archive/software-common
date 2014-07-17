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
        Blob::IntrusionMode intrusionMode = Blob::INTRUSION_DEFAULT;
        if(objectPath[0] == "board") {
                thresholded = Thresholder(normalized).simpleRGB(Vec3b(85, 85, 85), Vec3b(0, 128, 128));
	        erode(thresholded, thresholded, cv::Mat::ones(5,5,CV_8UC1));
        } else if(objectPath[0] == "empty_cell") {
                thresholded = Thresholder(normalized).simpleRGB(Vec3b(0, 0, 255), Vec3b(255, 0, 0), 21, -10);
                intrusionMode = Blob::INTRUSION_IGNORE;
        } else if(objectPath[0] == "peg") {
                thresholded = Thresholder(normalized).simpleRGB(Vec3b(0, 0, 255), Vec3b(255, 0, 0), 21, -10);
                intrusionMode = Blob::INTRUSION_SELECT;
        } else {
                throw std::runtime_error("Invalid object path");
        }
        /*
        erode(thresholded, thresholded, cv::Mat::ones(3,3,CV_8UC1)); // -1
        */
        Blob blob(thresholded, 300, 1e10, 1e10, false, true, intrusionMode);
        
        if(objectPath[0] != "board") {
            for(unsigned int i = 0; i < blob.data.size(); )
                if(blob.data[i].circularity < .7)
                        blob.data.erase(blob.data.begin()+i);
                else
                        i++;
            
            if(blob.data.size() > 8 && blob.data.front().radius > 2 * blob.data[4].radius) {
                // get rid of pipes being found
                blob.data.erase(blob.data.begin());
            }
        } else {
            Mat thresholded2 = Mat::zeros(img.image.rows, img.image.cols, CV_8UC1);
            for(unsigned int i = 0; i < blob.data.size(); i++) {
		        drawContours(thresholded2, std::vector<std::vector<cv::Point> >(1, blob.data[i].contour), 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
            }
            thresholded = thresholded2.clone();
            dilate(thresholded2, thresholded2, cv::Mat::ones(25,25,CV_8UC1));
            erode(thresholded2, thresholded2, cv::Mat::ones(25,25,CV_8UC1));
            blob = Blob(thresholded2, 300, 1e10, 1e10, false, true);
        }

        Mat res = normalized.clone();
        //if(blob.data.size() > 2)
        //        blob.data.resize(2);
        if(objectPath[0] == "peg") {
            thresholded *= 0;
        }
        if(objectPath[0] != "peg") {
            blob.drawResult(res, CV_RGB(0, 255, 0));
        } else {
            thresholded *= .5;
        }

        // Prepare results
        vector<property_tree::ptree> resultVector;
        BOOST_FOREACH(Blob::BlobData item, blob.data) {
                if(objectPath[0] == "peg") {
                    Mat x = item.intrusions.clone();
                    floodFill(x, Point(0, 0), CV_RGB(255, 255, 255));
                    bitwise_not(x, x);
                    x |= item.intrusions;
        	    erode(x, x, cv::Mat::ones(7,7,CV_8UC1));
        	    dilate(x, x, cv::Mat::ones(7,7,CV_8UC1));
                    thresholded |= x;
                    Blob blob(x, 300, 1e10, 1e10, false, true);
                    if(!blob.data.size()) continue;
                    blob.data.resize(1);
                    item = blob.data[0];
                    blob.drawResult(res, CV_RGB(0, 255, 0));
                }
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
