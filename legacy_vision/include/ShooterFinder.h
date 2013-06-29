#ifndef SHOOTER_FINDER_H
#define SHOOTER_FINDER_H

#include "IFinder.h"

class ShooterFinder : public IFinder
{
	public:
		ShooterFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 2 || !(
					objectPath[0] == "red" ||
					objectPath[0] == "blue" ||
			false) || !(
					objectPath[1] == "box" ||
					objectPath[1] == "small" ||
					objectPath[1] == "large" ||
			false))
				throw std::runtime_error("invalid objectPath");
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
};

#endif
