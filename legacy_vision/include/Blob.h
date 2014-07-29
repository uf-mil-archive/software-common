#ifndef BLOB_H
#define BLOB_H

#include <vector>

#include <boost/optional.hpp>

#include <opencv/cv.h>


class Blob
{
	public:
		struct BlobData {
			float area;
			float perimeter;
			cv::Point2f centroid;
			float radius;
			cv::Point2f rect_center;
			cv::Point2f direction;
			float aspect_ratio;  // ratio of length along angle to length perpendicular to angle. guaranteed to be >= 1
			bool is_vertical; // long axis is within 45deg of vertical
			float circularity; // [0, 1]
			float circularity_not_hull;
			float short_length;
			float long_length;
			float hollowness;
			std::vector<cv::Point> contour;
			cv::Mat intrusions;
			std::vector<cv::Point> approx_contour;

			bool operator==(const BlobData &bdata) const {
				return area == bdata.area;
			}

			bool operator<(const BlobData &bdata) const {
				return area < bdata.area;
			}
		};

		enum IntrusionMode {
			INTRUSION_DEFAULT,
			INTRUSION_IGNORE,
			INTRUSION_SELECT};
		std::vector<BlobData> data;
		Blob(const cv::Mat &img, float minContour, float maxContour, float maxPerimeter, bool sortByRadius=false, bool allowInternal=false, IntrusionMode intrusionMode=INTRUSION_DEFAULT);
		void drawResult(cv::Mat &img, const cv::Scalar &color);
};

#endif
