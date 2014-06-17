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

        Thresholder thresholder(blurred);       
        Mat dbg = thresholder.black();

        Blob blob(dbg, 300, 1e10, 1e10, false, true);
        for(unsigned int i = 0; i < blob.data.size(); )
                if(blob.data[i].circularity < .8)
                        blob.data.erase(blob.data.begin()+i);
                else
                        i++;

        Mat res = blurred.clone();

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

        return FinderResult(resultVector, res, dbg);
}
