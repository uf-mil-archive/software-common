#include <boost/foreach.hpp>

#include "Normalizer.h"
#include "Thresholder.h"
#include "Blob.h"

#include "HedgeFinder.h"

using namespace boost;
using namespace cv;
using namespace std;

IFinder::FinderResult HedgeFinder::find(const subjugator::ImageSource::Image &img) {
	// blur the image to remove noise
	//GaussianBlur(ioimages->prcd,ioimages->prcd,Size(3,3),10,15,BORDER_DEFAULT);

	// call to normalizer here
	Mat normalized = Normalizer::normRGB(img.image);

	// call to thresholder here
	Mat dbg = Thresholder(normalized).green();
	erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1));
	dilate(dbg, dbg, cv::Mat::ones(7,7,CV_8UC1));

	Blob blob(dbg, 1000, 100000, 20000);

	Mat res = img.image.clone();
	blob.drawResult(res, CV_RGB(0, 0, 255));

	// send average centroid of blobs, but with tiny scale so we servo until we can recognize the structure
	// if it only sees a reflection or some object other than the hedge, this could cause problems
	// another strategy succeeding will override this result
	Point sum(0, 0);
	float denom = 0;
	BOOST_FOREACH(const Blob::BlobData &data, blob.data) {
		sum += data.centroid*data.area;
		denom += data.area;
	}
	if(denom == 0) return FinderResult(vector<property_tree::ptree>(), res, dbg); // we see nothing at all
	Point center = 1/denom*sum;
	float scale = -1;

	// find bottom bar. assumes that more than one matching blob is never detected
	Point bottom_center;
	bool found_bottom = false;
	BOOST_FOREACH(const Blob::BlobData &data, blob.data)
		if(!data.is_vertical && 12 < data.aspect_ratio && data.aspect_ratio < 19) {
			bottom_center = data.centroid;
			found_bottom = true;
		}
	if(found_bottom) {
		// find y coordinate from maximum y centroid of vertical bars that are above the bottom bar
		float center_y = -1000;
		bool found_side = false;
		BOOST_FOREACH(const Blob::BlobData &data, blob.data)
			if(data.is_vertical && 4 < data.aspect_ratio && data.aspect_ratio < 10 &&
					data.centroid.y < bottom_center.y && data.centroid.y > center_y) {
				center_y = data.centroid.y;
				found_side = true;
			}
		if(found_side) {
			center = Point(bottom_center.x, center_y);
			scale = bottom_center.y - center_y;
		}
	} else {
		// try to find center from average of two side bar centers
		vector<Point> centers;
		BOOST_FOREACH(const Blob::BlobData &data, blob.data)
			if(data.is_vertical && 4 < data.aspect_ratio && data.aspect_ratio < 10)
				centers.push_back(data.centroid);
		if(centers.size() == 2) {
			center = 0.5*(centers[0] + centers[1]);
			scale = norm(centers[1] - centers[0])/2;
		}
	}
	if(scale < 0) return FinderResult(vector<property_tree::ptree>(), res, dbg);

	circle(res,center,scale,Scalar(255,255,255),5);

	// Prepare results
	vector<property_tree::ptree> resultVector;
	property_tree::ptree fResult;
	fResult.put_child("center", Point_to_ptree(center, img.image.size()));
	fResult.put("scale", scale);
	resultVector.push_back(fResult);
	return FinderResult(resultVector, res, dbg);
}

