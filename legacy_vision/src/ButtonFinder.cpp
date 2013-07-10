#include <boost/foreach.hpp>

#include "ButtonFinder.h"
#include "Blob.h"
#include "Normalizer.h"
#include "Thresholder.h"

using namespace boost;
using namespace cv;

IFinder::FinderResult ButtonFinder::find(const subjugator::ImageSource::Image &img) {
	// blur the image to remove noise
	Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 1.5);
	// call to normalizer here
	Mat normalized = Normalizer::normRGB(blurred);

	Thresholder thresholder(normalized);
	Mat dbg = thresholder.orange();

	// call to specific member function here
	Blob blob(dbg, 0, 1e100, 1e100);

	for(unsigned int i = 0; i < blob.data.size(); )
		if(blob.data[i].circularity < .5)
			blob.data.erase(blob.data.begin()+i);
		else
			i++;
	if(blob.data.size() > 2)
		blob.data.resize(2);
	if(blob.data.size() == 2 && blob.data[1].radius < blob.data[0].radius/2) {
		blob.data.resize(1);
	}

	// Draw result
	Mat res = img.image.clone();
	blob.drawResult(res, CV_RGB(0, 200, 0));

	vector<property_tree::ptree> resultVector;
	BOOST_FOREACH(const Blob::BlobData &b, blob.data) {
		// Prepare results
		property_tree::ptree fResult;
		fResult.put_child("center", Point_to_ptree(b.centroid, img));
		fResult.put("scale", b.area);

		resultVector.push_back(fResult);
	}

	return FinderResult(resultVector, res, dbg);
}
