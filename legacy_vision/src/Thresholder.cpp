#include "Thresholder.h"

using namespace cv;

Thresholder::Thresholder(const Mat &img) : img(img) {
	split(img, channelsRGB);

	cv::Mat imgHSV; cvtColor(img, imgHSV, CV_BGR2HSV);
	split(imgHSV, channelsHSV);

	cv::Mat imgLAB; cvtColor(img, imgLAB, CV_RGB2Lab);
	split(imgLAB, channelsLAB);
}

Mat Thresholder::orange() {
	Mat dbg;
	Mat b; adaptiveThreshold(channelsLAB[2],b,255,0,THRESH_BINARY_INV,201,13); // use lab channel hack --  higher offset = less yellow
	add(b,channelsRGB[2],dbg); // combine with red channel
	Mat v; inRange(channelsHSV[2],Scalar(0,0,0,0),Scalar(90,0,0,0),v); // filter out blacks
	subtract(dbg,v,dbg); // filter out blacks
	Mat s; inRange(channelsHSV[1],Scalar(0,0,0,0),Scalar(65,0,0,0),s);
	subtract(dbg,s,dbg); // filter whites
	threshold(dbg,dbg,175,255,THRESH_BINARY);
	return dbg;
}

Mat Thresholder::shooterRed() {
	Mat dbg;
	//res = prcd.clone();	
	Mat b; adaptiveThreshold(channelsLAB[2],b,255,0,THRESH_BINARY_INV,501,3); // use lab channel hack --  higher offset = less yellow
	add(b,channelsRGB[2],dbg); // combine with red channel
	//inRange(channelsRGB[1],Scalar(0,0,0,0),Scalar(50,0,0,0),dbg); // filter out blacks
	//subtract(dbg,channelsRGB[1],dbg); // filter out blacks
	Mat s; inRange(channelsHSV[1],Scalar(0,0,0,0),Scalar(30,0,0,0),s);
	subtract(b,s,dbg); // filter whites
	
	
	Mat blue; adaptiveThreshold(channelsHSV[0],blue,255,0,THRESH_BINARY,601,-8); // works well over [-7,-9]
	subtract(dbg, blue, dbg);
	
	dilate(dbg,dbg,cv::Mat::ones(5,5,CV_8UC1));
	erode(dbg,dbg,cv::Mat::ones(5,5,CV_8UC1));
	return dbg;
}

Mat Thresholder::red() {
	Mat dbg;
	Mat b; adaptiveThreshold(channelsLAB[2],b,255,0,THRESH_BINARY_INV,251,10); // use lab channel hack
	add(b,channelsRGB[2],dbg); // combine with red channel
	Mat v; inRange(channelsHSV[2],Scalar(0,0,0,0),Scalar(120,0,0,0),v); // filter out blacks
	subtract(dbg,v,dbg); // filter out blacks
	subtract(dbg,channelsRGB[1],dbg); // filter white/green/yellow
	adaptiveThreshold(dbg,dbg,255,0,THRESH_BINARY,201,-15);
	return dbg;
}

Mat Thresholder::yellow() {
	Mat dbg;
	// find whites (and hope for no washout!)
	Mat a; adaptiveThreshold(channelsLAB[1],a,255,0,THRESH_BINARY_INV,501,3);
	//subtract(dbg,channelsRGB[1],dbg);
	bitwise_and(a,channelsRGB[2],dbg); // and with red channel
	Mat s; inRange(channelsHSV[1],Scalar(0,0,0,0),Scalar(40,0,0,0),s);
	subtract(dbg,s,dbg); // remove whites
	adaptiveThreshold(dbg,dbg,255,0,THRESH_BINARY,171,-10);
	erode(dbg,dbg,cv::Mat::ones(7,7,CV_8UC1));
	dilate(dbg,dbg,cv::Mat::ones(7,7,CV_8UC1));
	return dbg;
}

Mat Thresholder::green() {
	Mat dbg;
	add(channelsLAB[1], channelsHSV[0], dbg);
	adaptiveThreshold(dbg,dbg,255,0,THRESH_BINARY_INV,71,4); // used incorrectly, but seems to work very robustly!
	Mat b; adaptiveThreshold(channelsLAB[2],b,255,0,THRESH_BINARY_INV,201,30);
	subtract(dbg,b,dbg);
	erode(dbg,dbg,cv::Mat::ones(9,9,CV_8UC1));
	dilate(dbg,dbg,cv::Mat::ones(7,7,CV_8UC1));
	return dbg;
}

Mat Thresholder::blue() {
	Mat dbg;
	Mat x; addWeighted(channelsHSV[0], 0.5, channelsLAB[1], 0.5, 0, x);
	adaptiveThreshold(channelsHSV[0],dbg,255,0,THRESH_BINARY,601,-8); // works well over [-7,-9]
	dilate(dbg,dbg,cv::Mat::ones(5,5,CV_8UC1));
	erode(dbg,dbg,cv::Mat::ones(9,9,CV_8UC1));
	dilate(dbg,dbg,cv::Mat::ones(5,5,CV_8UC1));
	return dbg;
}

Mat Thresholder::black() {
	Mat dbg;
	Mat v; channelsHSV[2].convertTo(v, CV_32FC1, 1/256., 1/256./2);
	log(v, v);
	v.convertTo(dbg, CV_8UC1, 40, 256);
	adaptiveThreshold(dbg, dbg, 255, 0, THRESH_BINARY_INV, 21, 3);
	erode(dbg,dbg,cv::Mat::ones(3,3,CV_8UC1));
	dilate(dbg,dbg,cv::Mat::ones(5,5,CV_8UC1));
	erode(dbg,dbg,cv::Mat::ones(3,3,CV_8UC1));
	return dbg;
}

static Vec3f divide(Vec3f a, Vec3f b) {
	return Vec3f(a[0]/b[0], a[1]/b[1], a[2]/b[2]);
}
static Vec3f multiply(Vec3f a, Vec3f b) {
	return Vec3f(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}
Mat Thresholder::forrest(Vec3b bg, Vec3b fg, double radius, double inclusion) {
	Mat blurred; GaussianBlur(img, blurred, Size(0, 0), radius, radius);
	
	Mat result = channelsRGB[0].clone(); // XXX
	for(int i = 0; i < img.rows; i++) {
		for(int j = 0; j < img.cols; j++) {
			Vec3f sample = blurred.at<Vec3b>(i, j);
			
			Vec3f light = divide(sample, .5*Vec3f(bg) + .5*Vec3f(fg)); // assuming near border of foreground and background
			//if(norm(1/norm(light)*light - 1/norm(Vec3f(1,1,1))*Vec3f(1,1,1)) > .3) {
			//	result.at<uint8_t>(i, j) = 0;
			//	continue;
			//}
			//light = divide(sample, Vec3f(bg));
			Vec3f predicted_sample_if_bg = multiply(light, bg);
			Vec3f predicted_sample_if_fg = multiply(light, fg);
			
			Vec3f sample2 = img.at<Vec3b>(i, j);
			Vec3f vec = (predicted_sample_if_fg - predicted_sample_if_bg);
			vec /= pow(norm(vec), 2);
			double score = (sample2 - predicted_sample_if_bg).dot(vec);
			if(score >= inclusion) {
				result.at<uint8_t>(i, j) = 255;
			} else {
				result.at<uint8_t>(i, j) = 0;
			}
			//result.at<uint8_t>(i, j) = norm(sample - predicted_sample_if_bg)/(norm(sample - predicted_sample_if_bg) + norm(sample - predicted_sample_if_fg))*255+.5;
		}
	}
	return result;
}
