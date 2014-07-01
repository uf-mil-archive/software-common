#include <boost/foreach.hpp>

#include <stdio.h>
#include <iostream>

#include "Blob.h"
#include "Contours.h"
#include "Normalizer.h"
#include "Thresholder.h"

#include "ShooterFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

IFinder::FinderResult ShooterFinder::find(const subjugator::ImageSource::Image &img) {
        Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 2);

        vector<property_tree::ptree> resultVector;
        Mat res = blurred.clone();
        Mat dbg;
        if(objectPath[0] == "board") {
                Mat normalized = Normalizer::normRGB(blurred);
                dbg = Thresholder(normalized).simpleRGB(Vec3b(106, 106, 43), Vec3b(0, 255, 0), 21, -3);
	        dilate(dbg, dbg, cv::Mat::ones(11,11,CV_8UC1));
	        erode(dbg, dbg, cv::Mat::ones(11,11,CV_8UC1));
                Contours contours(dbg, 100, 1e12, 1e12);
                contours.drawResult(res, CV_RGB(255, 255, 255));
                
                BOOST_FOREACH(const Contours::OuterBox &box, contours.boxes) {
                    property_tree::ptree fResult;
                    fResult.put_child("center", Point_to_ptree(box.centroid, img));
                    //fResult.put_child("direction", Direction_to_ptree(box.centroid, src[1] - src[0], img));
                    //fResult.put("direction_symmetry", 2);

                    fResult.put("orientation_error", box.orientationError);
                    fResult.put("angle", box.angle);
                    fResult.put("scale", box.area);
                    resultVector.push_back(fResult);
                }
        } else {
                dbg = Thresholder(blurred).black();
                Blob blob(dbg, 300, 1e10, 1e10, false, true);
                for(unsigned int i = 0; i < blob.data.size(); )
                        if(blob.data[i].circularity < .8)
                                blob.data.erase(blob.data.begin()+i);
                        else
                                i++;
                blob.drawResult(res, CV_RGB(0, 255, 0));
                BOOST_FOREACH(const Blob::BlobData &item, blob.data) {
                        property_tree::ptree fResult;
                        fResult.put_child("center", Point_to_ptree(item.rect_center, img));
                        fResult.put_child("direction", Direction_to_ptree(item.rect_center, item.direction, img));
                        fResult.put("direction_symmetry", 2);
                        resultVector.push_back(fResult);
                }
        }

        return FinderResult(resultVector, res, dbg);
}
