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
  Mat blurred; GaussianBlur(img.image, blurred, Size(0, 0), 3);

  Mat normalized = Normalizer::normRGB(blurred);

  Mat thresholded = Thresholder(normalized).red();
  
  //erode(thresholded, thresholded, cv::Mat::ones(13,13,CV_8UC1)); // -6
  dilate(thresholded, thresholded, cv::Mat::ones(3,3,CV_8UC1)); // +7
  //erode(thresholded, thresholded, cv::Mat::ones(3,3,CV_8UC1)); // -1

  Blob blob(thresholded, 1000, 1e10, 1e10);

  Mat res = img.image.clone();
  while(blob.data.size() && !blob.data[0].is_vertical)
    blob.data.erase(blob.data.begin());
  if(blob.data.size() > 1)
    blob.data.resize(1);
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

  return FinderResult(resultVector, res, thresholded);
}
