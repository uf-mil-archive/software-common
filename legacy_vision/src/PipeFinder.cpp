#include <boost/foreach.hpp>

#include "Line.h"
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
	erode(orange, orange, cv::Mat::ones(3,3,CV_8UC1));
	dilate(orange, orange, cv::Mat::ones(7,7,CV_8UC1));
	erode(orange, orange, cv::Mat::ones(7,7,CV_8UC1));

	Line line(2, config, orange);

	Mat res = img.image.clone();
	line.drawResult(res);

	// Prepare results
	vector<property_tree::ptree> resultVector;
	BOOST_FOREACH(const AvgLine &avgline, line.avgLines) {
		property_tree::ptree fResult;
		fResult.put_child("center", Point_to_ptree(avgline.centroid, img.image.size()));
		fResult.put("angle", avgline.angle);
		fResult.put("scale", avgline.length);
		resultVector.push_back(fResult);
	}

	return FinderResult(resultVector, res, orange);
}
