#ifndef GRAPES_FINDER_H
#define GRAPES_FINDER_H

#include "IFinder.h"

class GrapesFinder : public IFinder
{
	public:
		GrapesFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 1 || !(
					objectPath[0] == "board" ||
					objectPath[0] == "grape" ||
					objectPath[0] == "grape_close" ||
			false))
				throw std::runtime_error("invalid objectPath");
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
};

#endif
