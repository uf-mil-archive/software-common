#include <boost/foreach.hpp>

#include <stdio.h>
#include <iostream>

#include "Blob.h"
#include "Contours.h"
#include "Normalizer.h"
#include "Thresholder.h"

#include "BoatDockFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

IFinder::FinderResult BoatDockFinder::find(const subjugator::ImageSource::Image &img) {
        Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 1);

        vector<property_tree::ptree> resultVector;
        Mat res = blurred.clone();
        Mat dbg = Thresholder(blurred).black();
        Blob blob(dbg, 100, 1e10, 1e10, false, true);
        
        if(objectPath[0] == "circle") {
                for(unsigned int i = 0; i < blob.data.size(); )
                        if(blob.data[i].circularity < .55)
                                blob.data.erase(blob.data.begin()+i);
                        else
                                i++;
        } else if(objectPath[0] == "triangle") {
                for(unsigned int i = 0; i < blob.data.size(); )
                        if(blob.data[i].approx_contour.size() != 3)
                                blob.data.erase(blob.data.begin()+i);
                        else
                                i++;
                for(unsigned int i = 0; i < blob.data.size(); )
                        if(blob.data[i].circularity < .35 || blob.data[i].circularity > .55)
                                blob.data.erase(blob.data.begin()+i);
                        else
                                i++;
        } else if(objectPath[0] == "cruciform") {
                for(unsigned int i = 0; i < blob.data.size(); )
                        if(blob.data[i].approx_contour.size() < 10 || blob.data[i].approx_contour.size() > 12)
                                blob.data.erase(blob.data.begin()+i);
                        else
                                i++;
                for(unsigned int i = 0; i < blob.data.size(); )
                        if(blob.data[i].circularity < .4)
                                blob.data.erase(blob.data.begin()+i);
                        else
                                i++;
        } else {
                assert(false);
        }
        
        blob.drawResult(res, CV_RGB(0, 255, 0));
        BOOST_FOREACH(const Blob::BlobData &item, blob.data) {
                property_tree::ptree fResult;
                fResult.put_child("center", Point_to_ptree(item.rect_center, img));
                fResult.put_child("direction", Direction_to_ptree(item.rect_center, item.direction, img));
                fResult.put("direction_symmetry", 2);
                resultVector.push_back(fResult);
        }

        return FinderResult(resultVector, res, dbg);
}
