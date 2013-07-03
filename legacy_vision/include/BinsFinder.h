#ifndef BINS_FINDER_H
#define BINS_FINDER_H

#include <sstream>

#include <opencv/highgui.h>

#include "IFinder.h"

class BinsFinder : public IFinder
{
	public:
		BinsFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 1 || !(
					objectPath[0] == "all" ||
					objectPath[0] == "single" ||
					objectPath[0] == "single_save" ||
			false))
				throw std::runtime_error("invalid objectPath");
			
			if(objectPath[0] == "single") {
                for(int box_index = 0; box_index < 4; box_index++) {
                    std::ostringstream ss;
                    ss << config.get<std::string>("imagePath") << "/" << names[box_index] << ".png";
                    templates[box_index] = cv::imread(ss.str());
                }
            }
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
	private:
        cv::Mat templates[4];
        static const std::string names[4];
};

#endif
