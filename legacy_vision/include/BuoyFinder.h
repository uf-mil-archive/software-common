#ifndef BUOY_FINDER_H
#define BUOY_FINDER_H

#include "IFinder.h"

class BuoyFinder : public IFinder
{
	public:
		BuoyFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 1 || !(
					objectPath[0] == "green" ||
					objectPath[0] == "red" ||
					objectPath[0] == "yellow" ||
			false))
				throw std::runtime_error("invalid objectPath");
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
};

#endif
