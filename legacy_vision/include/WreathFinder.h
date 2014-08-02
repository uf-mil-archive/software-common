#ifndef WREATH_FINDER_H
#define WREATH_FINDER_H

#include "IFinder.h"

class WreathFinder : public IFinder
{
	public:
		WreathFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 2 || !(
			        objectPath[0] == "board" ||
			        objectPath[0] == "bin" ||
					objectPath[0] == "moonrock" ||
					objectPath[0] == "cheese" ||
                        false) || !(
                                        objectPath[1] == "high" ||
                                        objectPath[1] == "low" ||
			false))
				throw std::runtime_error("invalid objectPath");
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
};

#endif
