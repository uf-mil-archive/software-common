#include <boost/foreach.hpp>

#include "AvgLine.h"

using namespace cv;

AvgLine::AvgLine(vector<Line> lines) {
	assert(lines.size() > 0);

	centroid = Point2f(0, 0); // compute average centroid
	length = 0; // and length
	Point2f direction(0, 0); // and length-weighted direction
	BOOST_FOREACH(const Line &line, lines) {
		centroid += (line.first + line.second)*(1./2/lines.size());
		length += norm(line.second - line.first)*(1./lines.size());
		if(line.second.y > line.first.y) // use direction that points more downward so angle will be in [0, pi]
			direction += line.second - line.first;
		else
			direction += line.first - line.second;
	}

	angle = atan2(direction.y, direction.x);
}
