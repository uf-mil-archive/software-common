#ifndef BINS_FINDER_H
#define BINS_FINDER_H

#include "IFinder.h"

class BinsFinder : public IFinder
{
	public:
		BinsFinder(std::vector<std::string> objectPath, boost::property_tree::ptree config) : IFinder(objectPath, config) {
			if(objectPath.size() != 1 || !(
					objectPath[0] == "all" ||
					objectPath[0] == "single" ||
			false))
				throw std::runtime_error("invalid objectPath");
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
};

#endif
