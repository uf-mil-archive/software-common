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
        Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 3);

        Mat normalized = Normalizer::normRGB(blurred);

        // call to thresholder here
        Mat thresholded = Thresholder(normalized).simpleRGB(Vec3b(0, 255, 255), Vec3b(255, 0, 0));
        erode(thresholded, thresholded, cv::Mat::ones(13,13,CV_8UC1)); // -6
        dilate(thresholded, thresholded, cv::Mat::ones(15,15,CV_8UC1)); // +7
        erode(thresholded, thresholded, cv::Mat::ones(3,3,CV_8UC1)); // -1

        Blob blob(thresholded, 300, 1e10, 1e10, false, true);
        for(unsigned int i = 0; i < blob.data.size(); )
                if(blob.data[i].circularity < .8)
                        blob.data.erase(blob.data.begin()+i);
                else
                        i++;


        Mat res = img.image.clone();
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
                resultVector.push_back(fResult);
        }

        return FinderResult(resultVector, res, thresholded);
}
