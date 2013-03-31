#ifndef OBJ_FINDING_H
#define OBJ_FINDING_H

// triangles must not overlap when projected onto image, which essentially means
// that they should be planar

#include <vector>
#include <string>

#include <boost/foreach.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "image.h"


struct Triangle {
    Eigen::Vector3d corners[3];
    Triangle(Eigen::Vector3d c0, Eigen::Vector3d c1, Eigen::Vector3d c2) {
        corners[0] = c0;
        corners[1] = c1;
        corners[2] = c2;
    }
    Eigen::Vector3d centroid() const {
        return (corners[0] + corners[1] + corners[2])/3;
    }
    double area() const {
        Eigen::Vector3d x = corners[1] - corners[0];
        Eigen::Vector3d y = corners[2] - corners[0];
        return .5*sqrt(
            pow(x[1]*y[2] - x[2]*y[1], 2) +
            pow(x[2]*y[0] - x[0]*y[2], 2) +
            pow(x[0]*y[1] - x[1]*y[0], 2));
    }
};

struct Component {
    std::string name;
    std::vector<Triangle> triangles;
    Eigen::Vector3d center_of_mass() const {
        // assumes volumes aren't filled - only surfaces exist
        Eigen::Vector3d total_centroid = Eigen::Vector3d::Zero();
        double total_area = 0;
        BOOST_FOREACH(const Triangle &tri, triangles) {
            total_centroid += tri.centroid()*tri.area();
            total_area += tri.area();
        }
        return total_centroid/total_area;
    }
};

struct Marker {
    std::string name;
    Eigen::Vector3d position;
};

struct Obj {
    std::vector<Component> components;
    std::vector<Marker> markers;
    
    Obj(const std::string filename);
    
    void query(const TaggedImage &image, Eigen::Vector3d pos, Eigen::Quaterniond orientation, std::vector<Result> &results, std::vector<int>* dbg_image=NULL) const;
};

#endif
