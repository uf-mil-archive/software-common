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
			cv::Point centroid;
			float radius;
			float angle; // angle of longest axis, in radians. 0 = horizontal. more counterclockwise is more positive
			float aspect_ratio;  // ratio of length along angle to length perpendicular to angle. guaranteed to be >= 1
			bool is_vertical; // long axis is within 45deg of vertical
			float circularity; // [0, 1]
			std::vector<cv::Point> contour;

			bool operator==(const BlobData &bdata) const {
				return area == bdata.area;
			}

			bool operator<(const BlobData &bdata) const {
				return area < bdata.area;
			}
		};

		std::vector<BlobData> data;
		Blob(const cv::Mat &img, float minContour, float maxContour, float maxPerimeter);
		void drawResult(cv::Mat &img, const cv::Scalar &color);
};

#endif
