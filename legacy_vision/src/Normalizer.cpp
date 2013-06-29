#include "Normalizer.h"

using namespace cv;

Mat Normalizer::normRGB(const Mat &img) {
	Mat img2 = img.clone();

	// NORMALIZED RGB
	for(int i = 0; i < img.rows; i++)
		for(int j = 0; j < img.cols; j++) {
			Vec3b rgb_vec = img.at<Vec3b>(i,j);
			double sum = (double)(rgb_vec[0]+rgb_vec[1]+rgb_vec[2]+0.001);

			for(int k=0; k < img.channels(); k++)
				img2.at<Vec3b>(i,j)[k] = (double)img.at<Vec3b>(i,j)[k] / sum * 255; 
		}

	return img2;
}
