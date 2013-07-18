#ifndef SHOOTER_FINDER_H
#define SHOOTER_FINDER_H

#include "IFinder.h"
#include <boost/optional.hpp>

class ShooterFinder : public IFinder
{
	public:
		ShooterFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 2 || !(
					objectPath[0] == "red" ||
                                        objectPath[0] == "yellow" ||
                                        objectPath[0] == "green" ||                                        
					objectPath[0] == "blue" ||
			false) || !(
					objectPath[1] == "box" ||
					objectPath[1] == "small" ||
					objectPath[1] == "large" ||
			false))
				throw std::runtime_error("invalid objectPath");
		};
		FinderResult find(const subjugator::ImageSource::Image &img);

	private:        
                struct QuadPointResults {
                    cv::Point point;
                    int score;
		    cv::Vec<uchar, 4> hues;
		    cv::Vec<uchar, 4> sats;
		};
		boost::optional<QuadPointResults> trackQuadPoint(
			const cv::Mat (&hsv_split)[3]);
                std::pair<cv::Vec<uchar, 4>, cv::Vec<uchar, 4> > sample_point(
                    const cv::Mat (&hsv_split)[3], int r, int c, int offset);
};

#endif
