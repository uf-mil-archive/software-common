#include <boost/foreach.hpp>
#include <boost/optional.hpp>

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
	Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 5, 5);

	// call to normalizer here
	Mat normalized = Normalizer::normRGB(blurred);

	// call to thresholder here
	Mat dbg = Thresholder(normalized).green();
	erode(dbg, dbg, cv::Mat::ones(3,3,CV_8UC1)); // -1
	dilate(dbg, dbg, cv::Mat::ones(7,7,CV_8UC1)); // +3
	erode(dbg, dbg, cv::Mat::ones(7,7,CV_8UC1)); // -2

	Blob blob(dbg, 100, 100000, 20000);

	Mat res = normalized.clone();
	blob.drawResult(res, CV_RGB(0, 0, 255));

	// send average centroid of blobs, but with tiny scale so we servo until we can recognize the structure
	// if it only sees a reflection or some object other than the hedge, this could cause problems
	// another strategy succeeding will override this result
	Point2f sum(0, 0);
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
	const Blob::BlobData *bottom;
	bool found_bottom = false;
	BOOST_FOREACH(const Blob::BlobData &data, blob.data)
		if(!data.is_vertical && 5 < data.aspect_ratio && data.aspect_ratio < 50) {
			bottom_center = data.centroid;
			found_bottom = true;
			bottom = &data;
			break;
		}
	optional<cv::Point3d> forward;
	if(found_bottom) {
		// find y coordinate from maximum y centroid of vertical bars that are above the bottom bar
		float center_y = -1000;
		bool found_side = false;
		BOOST_FOREACH(const Blob::BlobData &data, blob.data)
			if(data.is_vertical && 3 < data.aspect_ratio && data.aspect_ratio < bottom->aspect_ratio &&
					data.centroid.y < bottom_center.y && data.centroid.y > center_y) {
				center_y = data.centroid.y;
				found_side = true;
			}
		if(found_side) {
			center = Point(bottom_center.x, center_y);
			scale = bottom_center.y - center_y;
		}
		
		cv::Point3d ray1 = img.camera_model.projectPixelTo3dRay(bottom->rect_center + bottom->direction*(bottom->long_length/2));
		cv::Point3d ray2 = img.camera_model.projectPixelTo3dRay(bottom->rect_center - bottom->direction*(bottom->long_length/2));
		double desired_y = (ray1.y + ray2.y)/2;
		if(ray1.y != 0 && ray2.y != 0) {
		    ray1 *= desired_y / ray1.y;
		    ray2 *= desired_y / ray2.y;
		    cv::Point3d x = ray2 - ray1; // x.y is 0
		    cv::Point3d perp(x.z, x.y, -x.x);
		    if(perp.z < 0) perp = -perp;
		    perp *= 1/norm(perp);
		    forward = perp;
	    }
	}
	if(scale < 0) return FinderResult(vector<property_tree::ptree>(), res, dbg);

	circle(res,center,scale,Scalar(255,255,255),5);

	// Prepare results
	vector<property_tree::ptree> resultVector;
	property_tree::ptree fResult;
	fResult.put_child("center", Point_to_ptree(center, img));
	fResult.put("scale", scale);
	if(forward) {
    	fResult.put_child("forward", raw_to_ptree(*forward));
	}
	resultVector.push_back(fResult);
	return FinderResult(resultVector, res, dbg);
}

