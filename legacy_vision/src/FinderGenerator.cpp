#include <iostream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>

#include "FinderGenerator.h"

#include "BuoyFinder.h"
#include "GrapesFinder.h"
#include "PipeFinder.h"
#include "HedgeFinder.h"
#include "ShooterFinder.h"
#include "BinsFinder.h"
#include "WreathFinder.h"

using namespace std;
using namespace boost;

vector<pair<string, shared_ptr<IFinder> > > FinderGenerator::buildFinders(const vector<string> &objectNames, const property_tree::ptree &config) {
	vector<pair<string, shared_ptr<IFinder> > > finders;
	BOOST_FOREACH(const string &objectName, objectNames) {
		vector<string> objectPath; split(objectPath, objectName, is_any_of("/"));
		if(objectPath.size() == 0)
			throw runtime_error("empty objectName");

		vector<string> rest(objectPath.begin()+1, objectPath.end());
		property_tree::ptree fconfig = config.get_child(objectPath[0], property_tree::ptree());
		shared_ptr<IFinder> finder =
			objectPath[0] == "buoy" ? make_shared<BuoyFinder>(rest, fconfig) :
			objectPath[0] == "grapes" ? make_shared<GrapesFinder>(rest, fconfig) :
			objectPath[0] == "pipe" ? make_shared<PipeFinder>(rest, fconfig) :
			objectPath[0] == "hedge" ? make_shared<HedgeFinder>(rest, fconfig) :
			objectPath[0] == "shooter" ? make_shared<ShooterFinder>(rest, fconfig) :
			objectPath[0] == "bins" ? make_shared<BinsFinder>(rest, fconfig) :
			objectPath[0] == "wreath" ? make_shared<WreathFinder>(rest, fconfig) :
			shared_ptr<IFinder>();
		if(!finder)
			throw runtime_error("unknown objectName: " + objectPath[0]);
		finders.push_back(make_pair(objectName, finder));
	}

	return finders;
}
