#ifndef BLOB_H
#define BLOB_H

#include <vector>

#include <opencv/cv.h>


class Blob
{
	public:
		struct BlobData {
			float area;
			float perimeter;
			cv::Point2f centroid;
			float radius;
			cv::Point2f direction;
			float aspect_ratio;  // ratio of length along angle to length perpendicular to angle. guaranteed to be >= 1
			bool is_vertical; // long axis is within 45deg of vertical
			float circularity; // [0, 1]
			float short_length;
			float long_length;
			std::vector<cv::Point> contour;

			bool operator==(const BlobData &bdata) const {
				return area == bdata.area;
			}

			bool operator<(const BlobData &bdata) const {
				return area < bdata.area;
			}
		};

		std::vector<BlobData> data;
		Blob(const cv::Mat &img, float minContour, float maxContour, float maxPerimeter, bool sortByRadius=false);
		void drawResult(cv::Mat &img, const cv::Scalar &color);
};

#endif
