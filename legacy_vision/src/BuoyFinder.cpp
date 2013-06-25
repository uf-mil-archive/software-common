#include <boost/foreach.hpp>

#include "BuoyFinder.h"
#include "Blob.h"
#include "Normalizer.h"
#include "Thresholder.h"

using namespace boost;
using namespace cv;

IFinder::FinderResult BuoyFinder::find(const subjugator::ImageSource::Image &img) {
	// blur the image to remove noise
	Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 1.5);
	// call to normalizer here
	Mat normalized = Normalizer::normRGB(blurred);

	Thresholder thresholder(normalized);
	Mat dbg = objectPath[0] == "red" ? thresholder.orange() : thresholder.green(); // use green thresholder for both green and yellow

	// call to specific member function here
	Blob blob(dbg, config.get<float>("minContour"), config.get<float>("maxContour"), config.get<float>("maxPerimeter"));

	for(unsigned int i = 0; i < blob.data.size(); )
		if(blob.data[i].circularity < .5 || blob.data[i].centroid.y > 480*4/5)
			blob.data.erase(blob.data.begin()+i);
		else
			i++;
	if(blob.data.size() > 3)
		blob.data.resize(3);

	// Draw result
	Mat res = img.image.clone();
	blob.drawResult(res, CV_RGB(0, 200, 0));

	vector<property_tree::ptree> resultVector;
	BOOST_FOREACH(const Blob::BlobData &b, blob.data) {
		// Prepare results
		property_tree::ptree fResult;
		fResult.put_child("center", Point_to_ptree(b.centroid, img.image.size()));
		fResult.put("scale", b.area);
		
		// Check for color of blob
		Mat tempHSV; cvtColor(img.image,tempHSV,CV_BGR2HSV);
		std::vector<Mat> channelsHSV(img.image.channels()); split(tempHSV,channelsHSV);
		Mat tempMask = Mat::zeros(img.image.rows, img.image.cols, CV_8UC1);
			drawContours(tempMask, std::vector<std::vector<cv::Point> >(1, b.contour), 0, Scalar(255), CV_FILLED, 1, vector<Vec4i>(), 5);
		fResult.put("hue", mean(channelsHSV[0], tempMask)[0]);

		resultVector.push_back(fResult);
	}

	return FinderResult(resultVector, res, dbg);
}
