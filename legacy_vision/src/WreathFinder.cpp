#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include "Normalizer.h"
#include "Thresholder.h"
#include "Blob.h"

#include "WreathFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

IFinder::FinderResult WreathFinder::find(const subjugator::ImageSource::Image &img) {
	Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 3);

	Mat normalized = Normalizer::normRGB(img.image);

vector<property_tree::ptree> resultVector;
        Mat dbg;
        if(objectPath[0] == "moonrock") {
            dbg = Thresholder(normalized).red();
            //dilate(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
            //erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
            //dilate(dbg, dbg, cv::Mat::ones(5,5,CV_8UC1));
        } else if(objectPath[0] == "cheese") {
            dbg = Thresholder(normalized).green();
            dilate(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
            erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
            dilate(dbg, dbg, cv::Mat::ones(9,9,CV_8UC1));
        } else {
            throw std::runtime_error("Invalid object path");
        }

        Blob blob(dbg, 100, 1000000, 1000000);

        for(unsigned int i = 0; i < blob.data.size(); ) {
            if(objectPath[0] == "moonrock") {
                if(blob.data[i].circularity < .4 || blob.data[i].circularity > .8)
                    blob.data.erase(blob.data.begin()+i);
                else
                    i++;
            } else if(objectPath[0] == "cheese") {
                if(blob.data[i].circularity < .35 || blob.data[i].circularity > .8)
                    blob.data.erase(blob.data.begin()+i);
                else
                    i++;
            } else {
            throw std::runtime_error("Invalid object path");
            }
	    }
	    
	    for(unsigned int i = 0; i < blob.data.size(); ) {
            if(objectPath[0] == "moonrock") {
                if(blob.data[i].radius < 30 || blob.data[i].radius > 90)
                    blob.data.erase(blob.data.begin()+i);
                else
                    i++;;
            } else if(objectPath[0] == "cheese") {
                if(blob.data[i].radius < 25 || blob.data[i].radius > 80)
                    blob.data.erase(blob.data.begin()+i);
                else
                    i++;
            } else {
            throw std::runtime_error("Invalid object path");
            }
	    }
	/*vector<property_tree::ptree> resultVector;
	Mat dbg;
	dbg = Thresholder(normalized).simpleRGB(Vec3b(85, 85, 85), Vec3b(0, 0, 255), 11, -10);
	dilate(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
	erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
	//dilate(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));

	Blob blob(dbg, 100, 1000000, 1000000);

        for(unsigned int i = 0; i < blob.data.size(); )
            if(blob.data[i].circularity < .64 - .1 || blob.data[i].circularity > .64 + .1)
                blob.data.erase(blob.data.begin()+i);
            else
                i++;*/


	Mat res = normalized.clone();
	blob.drawResult(res, CV_RGB(255, 0, 0));

	BOOST_FOREACH(const Blob::BlobData &data, blob.data) {
//			if(1.15 < data.aspect_ratio && data.aspect_ratio < 1.5) {
			property_tree::ptree fResult;
			fResult.put_child("center", Point_to_ptree(data.centroid, img));
			fResult.put("scale", data.radius);
			fResult.put_child("direction", Direction_to_ptree(data.centroid, data.direction, img));
            fResult.put("direction_symmetry", 4);
			resultVector.push_back(fResult);
//			}
	}
	return FinderResult(resultVector, res, dbg);
}

