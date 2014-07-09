#ifndef SHOOTER_FINDER_H
#define SHOOTER_FINDER_H

#include "IFinder.h"

class BoatDockFinder : public IFinder
{
	public:
		BoatDockFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 1 || !(
					objectPath[0] == "board" ||
					objectPath[0] == "hole" ||
			false))
				throw std::runtime_error("invalid objectPath");
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
};

#endif
