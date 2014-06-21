#ifndef BINS_FINDER_H
#define BINS_FINDER_H

#include <sstream>
#include <vector>

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
			
			names.resize(4);
			names[0].push_back("1a"); names[0].push_back("1b");
			names[1].push_back("2a"); names[1].push_back("2b");
			names[2].push_back("3a"); names[2].push_back("3b");
			names[3].push_back("4");
			
			if(objectPath[0] == "single") {
		        int i = 0; BOOST_FOREACH(const std::vector<std::string> & subnames, names) {
		            templates.push_back(std::vector<cv::Mat>());
		            for(int j = 0; j < subnames.size(); j++) {
		                std::ostringstream ss;
		                ss << config.get<std::string>("imagePath") << "/" << names[i][j] << ".png";
				std::cout << ss.str();
		                templates[templates.size()-1].push_back(cv::imread(ss.str()));
		            }
		        i++; }
            }
		};
		FinderResult find(const subjugator::ImageSource::Image &img);
	private:
        std::vector<std::vector<cv::Mat> > templates;
        std::vector<std::vector<std::string> > names;
};

#endif
