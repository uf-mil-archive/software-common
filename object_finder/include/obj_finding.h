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
        Eigen::Vector3d a = corners[1] - corners[0];
        Eigen::Vector3d b = corners[2] - corners[0];
        return a.cross(b).norm()/2;
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

struct ResultWithArea : public Result {
    double area;
    ResultWithArea() { }
    ResultWithArea(Result result, double area) : Result(result), area(area) { }
    Eigen::Vector3d avg_color_assuming_unseen_is(Eigen::Vector3d unseen_color) {
        return (total_color+(area-count)*unseen_color)/area;
    }
};

struct Obj {
    std::vector<Component> components;
    std::vector<Marker> markers;
    
    Obj(const std::string filename);
    
    void query(const TaggedImage &image, Eigen::Vector3d pos, Eigen::Quaterniond orientation, std::vector<ResultWithArea> &results, std::vector<int>* dbg_image=NULL) const;
};

#endif
