#ifndef FINDER_GENERATOR_H
#define FINDER_GENERATOR_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include "IFinder.h"


class FinderGenerator
{
	public:
		static std::vector<std::pair<std::string, boost::shared_ptr<IFinder> > > buildFinders(const std::vector<std::string> &objectNames, const boost::property_tree::ptree &config);
};

#endif
