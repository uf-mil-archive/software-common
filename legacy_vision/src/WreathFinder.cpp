#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include "Normalizer.h"
#include "Contours.h"
#include "Thresholder.h"
#include "Blob.h"

#include "WreathFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

IFinder::FinderResult WreathFinder::find(const subjugator::ImageSource::Image &img) {
    Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 3);
    
    bool low = objectPath[1] == "low";
    double scale = low ? 2 : 1;

    Mat normalized = Normalizer::normRGB(img.image);

    Mat dbg;

    if(objectPath[0] == "board") {
        std::vector<cv::Mat> channelsRGB; split(normalized, channelsRGB);
        Mat dbg; adaptiveThreshold(channelsRGB[0],dbg,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,25,0);
        erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
        Contours contours(dbg, 10000, 1e12, 1e12, img.camera_model);
        Mat res = normalized.clone();
        contours.drawResult(res, CV_RGB(255, 255, 255));
        vector<property_tree::ptree> resultVector;
        BOOST_FOREACH(const Contours::OuterBox &box, contours.boxes) {
            if(box.aspect_ratio > 1.5) continue;
            property_tree::ptree fResult;
            fResult.put_child("center", Point_to_ptree(box.centroid, img));
            fResult.put_child("direction", Direction_to_ptree(box.centroid, box.corners[2] - box.corners[0], img));
            fResult.put("direction_symmetry", 4);

            fResult.put("orientation_error", box.orientationError);
            fResult.put("angle", box.angle);
            fResult.put("scale", box.area);
            resultVector.push_back(fResult);
        }
        return FinderResult(resultVector, res, dbg);
    }

    vector<property_tree::ptree> resultVector;
    if(objectPath[0] == "moonrock") {
        dbg = Thresholder(img.image).black2();
        //dbg = Thresholder(normalized).red();
        //dilate(dbg, dbg, cv::Mat::ones(5,5,CV_8UC1));
        //erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
        //dilate(dbg, dbg, cv::Mat::ones(5,5,CV_8UC1));
    } else if(objectPath[0] == "cheese") {
        dbg = Thresholder(img.image).black2();
        //dilate(dbg, dbg, cv::Mat::ones(5,5,CV_8UC1));
        //erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
        //dilate(dbg, dbg, cv::Mat::ones(9,9,CV_8UC1));
    } else {
        throw std::runtime_error("Invalid object path");
    }
    cv::bitwise_not(dbg, dbg);
    erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));

    Blob blob(dbg, 50, 1000000, 1000000, true, true);
    for(unsigned int i = 0; i < blob.data.size(); ) {
        if(blob.data[i].radius > scale * 70)
            blob.data.erase(blob.data.begin()+i);
        else
            i++;;
    }
    Mat res = img.image.clone();
    blob.drawResult(res, CV_RGB(255, 0, 0));
    /*for(unsigned int i = 0; i < blob.data.size(); ) {
        if(blob.data[i].hollowness < .3)
            blob.data.erase(blob.data.begin()+i);
        else
            i++;;
    }*/


    for(unsigned int i = 0; i < blob.data.size(); ) {
        if(blob.data[i].parent_area < 1000*scale*scale)
            blob.data.erase(blob.data.begin()+i);
        else
            i++;
    }
    for(unsigned int i = 0; i < blob.data.size(); ) {
        if(objectPath[0] == "moonrock") {
            if(blob.data[i].circularity_not_hull < .25 || blob.data[i].circularity_not_hull > .75)
                blob.data.erase(blob.data.begin()+i);
            else
                i++;
        } else if(objectPath[0] == "cheese") {
            if(blob.data[i].circularity_not_hull < .15 || blob.data[i].circularity_not_hull > .45)
                blob.data.erase(blob.data.begin()+i);
            else
                i++;
        } else {
            throw std::runtime_error("Invalid object path");
        }
    }
     
    for(unsigned int i = 0; i < blob.data.size(); ) {
        if(objectPath[0] == "moonrock") {
            if(blob.data[i].radius < scale * 6 || blob.data[i].radius > scale * 13*1.5)
                blob.data.erase(blob.data.begin()+i);
            else
                i++;;
        } else if(objectPath[0] == "cheese") {
            if(blob.data[i].radius < scale * 27/1.5 || blob.data[i].radius > scale * 27*1.5)
                blob.data.erase(blob.data.begin()+i);
            else
                i++;
        } else {
        throw std::runtime_error("Invalid object path");
        }
    }
    blob.drawResult(res, CV_RGB(0, 255, 0));

//dbg *= 0;

    BOOST_FOREACH(const Blob::BlobData &data, blob.data) {
//                        if(1.15 < data.aspect_ratio && data.aspect_ratio < 1.5) {
                    property_tree::ptree fResult;
                    fResult.put_child("center", Point_to_ptree(data.centroid, img));
                    fResult.put("scale", data.radius);
                    fResult.put_child("direction", Direction_to_ptree(data.centroid, data.direction, img));
        fResult.put("direction_symmetry", 2);
                    Mat tempMask = Mat::zeros(img.image.rows, img.image.cols, CV_8UC1);
                drawContours(tempMask, std::vector<std::vector<cv::Point> >(1, data.contour), 0, Scalar(255), 3, 8, vector<Vec4i>(), 5);
                //BOOST_FOREACH(std::vector<cv::Point> const & hole, data.holes) {
                //    drawContours(tempMask, std::vector<std::vector<cv::Point> >(1, hole), 0, Scalar(0), CV_FILLED, 8, vector<Vec4i>(), 5);
                //}
                //dbg |= tempMask;
                fResult.put("redness", mean(normalized, tempMask)[2]/mean(normalized, tempMask)[1]);
                resultVector.push_back(fResult);
//                        }
    }
    return FinderResult(resultVector, res, dbg);
}
