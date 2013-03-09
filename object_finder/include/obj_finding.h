#ifndef OBJ_FINDING_H
#define OBJ_FINDING_H

// triangles must not overlap when projected onto image, which essentially means
// that they should be planar

#include <vector>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "image.h"

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;


struct Triangle {
    Eigen::Vector3d corners[3];
    Triangle(Eigen::Vector3d c0, Eigen::Vector3d c1, Eigen::Vector3d c2) {
        corners[0] = c0;
        corners[1] = c1;
        corners[2] = c2;
    }
};

struct Component {
    string name;
    vector<Triangle> triangles;
};


struct Result {
    Eigen::Vector3d total_color;
    double count;
    
    Result() { }
    Result(Eigen::Vector3d total_color, double count) :
        total_color(total_color), count(count) {
    }
};

struct Obj {
    vector<Component> components;
    
    Obj(const vector<Component> &components) :
        components(components) {
    }
    
    static Obj from_file(string filename);
    
    void query(const TaggedImage &image, Eigen::Vector3d pos, Eigen::Quaterniond orientation, std::vector<Result> &results, std::vector<int>* dbg_image=NULL);
};

#endif
