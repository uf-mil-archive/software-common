#include <boost/foreach.hpp>

#include "Blob.h"
#include "Normalizer.h"
#include "Thresholder.h"

#include "PipeFinder.h"

using namespace cv;
using namespace boost;
using namespace std;

IFinder::FinderResult PipeFinder::find(const subjugator::ImageSource::Image &img) {
	Mat blurred; GaussianBlur(img.image, blurred, Size(3,3), 10, 15, BORDER_DEFAULT);

	Mat normalized = Normalizer::normRGB(blurred);

	// call to thresholder here
	Mat orange = Thresholder(normalized).orange();
	erode(orange, orange, cv::Mat::ones(13,13,CV_8UC1)); // -2
	dilate(orange, orange, cv::Mat::ones(7,7,CV_8UC1)); // +3
	//erode(orange, orange, cv::Mat::ones(7,7,CV_8UC1)); // -1

        Blob blob(orange, 1000, 1e10, 1e10);

	Mat res = img.image.clone();
        if(blob.data.size() > 2)
                blob.data.resize(2);
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

	return FinderResult(resultVector, res, orange);
}
