// triangles must not overlap when projected onto image, which essentially means
// that they should be planar

#include <sstream>
#include <fstream>

#include <boost/foreach.hpp>

#include "obj_finding.h"


using namespace std;
using namespace Eigen;
using namespace sensor_msgs;


Obj Obj::from_file(string filename) {
    vector<Vector3d> vertices;
    vector<Component> components;
    
    ifstream f(filename.c_str());
    if(!f.is_open()) {
        throw runtime_error("couldn't open file");
    }
    
    while(!f.eof()) {
        string line; getline(f, line);
        
        size_t hash_pos = line.find("#");
        if(hash_pos != string::npos) {
            line = line.substr(0, hash_pos);
        }
        
        size_t content_pos = line.find_first_not_of("\t ");
        if(content_pos == string::npos) {
            continue;
        }
        line = line.substr(content_pos);
        
        stringstream ss(line);
        
        string command; ss >> command;
        
        if(command == "o") {
            components.push_back(Component());
            ss >> components[components.size()-1].name;
        } else if(command == "v") {
            double x, y, z; ss >> x >> y >> z;
            vertices.push_back(Vector3d(x, y, z));
        } else if(command == "f") {
            int i0, i1, i2; ss >> i0 >> i1 >> i2;
            components[components.size()-1].triangles.push_back(Triangle(vertices[i0 - 1], vertices[i1 - 1], vertices[i2 - 1]));
        }
    }
    
    return Obj(components);
}


void Obj::query(const TaggedImage &image, Vector3d pos, Quaterniond orientation, vector<Result> &results, vector<int>* dbg_image) {
    results.resize(components.size());
    
    BOOST_FOREACH(const Component &component, components) {
        int j = &component - components.data();
    
        Vector3d &total_color = results[j].total_color;
        double &count = results[j].count;
        total_color = Vector3d::Zero();
        count = 0;
        
        BOOST_FOREACH(const Triangle &tri, component.triangles) {
            // XXX one corner being behind the camera does not mean the triangle is invisible!
            // instead, it is an "external triangle" that needs to be handled specially
            
            Vector3d c0_camera = image.transform.inverse() * (pos + orientation._transformVector(tri.corners[0]));
            Vector3d c0_homo = image.proj * c0_camera.homogeneous();
            if(c0_homo(2) <= 0) continue; // behind camera
            Vector2d c0 = c0_homo.hnormalized();
            
            Vector3d c1_camera = image.transform.inverse() * (pos + orientation._transformVector(tri.corners[1]));
            Vector3d c1_homo = image.proj * c1_camera.homogeneous();
            if(c1_homo(2) <= 0) continue; // behind camera
            Vector2d c1 = c1_homo.hnormalized();
            
            Vector3d c2_camera = image.transform.inverse() * (pos + orientation._transformVector(tri.corners[2]));
            Vector3d c2_homo = image.proj * c2_camera.homogeneous();
            if(c2_homo(2) <= 0) continue; // behind camera
            Vector2d c2 = c2_homo.hnormalized();
            
            // sort corners by y coordinate
            if(c1[1] < c0[1]) swap(c1, c0);
            if(c2[1] < c1[1]) swap(c2, c1);
            if(c1[1] < c0[1]) swap(c1, c0);
            
            for(int i = 0; i < 2; i++) {
                // i == 0 means between c0->c2 and c0->c1 edges
                // i == 1 means between c0->c2 and c1->c2 edges
                
                Vector2d ca, cb;
                if(i == 0) {
                    ca = c0;
                    cb = c1;
                } else {
                    ca = c1;
                    cb = c2;
                }
                
                // draw region between c0->c2 and ca->cb edges
                for(int Y = max(0, (int)ceil(ca[1] - 0.5)); Y + 0.5 < cb[1] && Y < (int)image.cam_info.height ; Y++) {
                    double y = Y + 0.5;
                    double x1 = (y - c0[1])*(c2[0] - c0[0])/(c2[1] - c0[1]) + c0[0];
                    double x2 = (y - ca[1])*(cb[0] - ca[0])/(cb[1] - ca[1]) + ca[0];
                    if(x2 < x1) swap(x2, x1); // sort x's
                    int X1 = max(0, min((int)image.cam_info.width-1, (int)ceil(x1 - 0.5)));
                    int X2 = max(0, min((int)image.cam_info.width-1, (int)ceil(x2 - 0.5)));
                    total_color +=
                        (X2 == 0 ? Vector3d::Zero() : image.sumimage[Y * image.cam_info.width + X2 - 1]) -
                        (X1 == 0 ? Vector3d::Zero() : image.sumimage[Y * image.cam_info.width + X1 - 1]);
                    count += X2 - X1;
                    if(dbg_image) {
                        for(int X = X1; X < X2; X++) {
                            (*dbg_image)[Y * image.cam_info.width + X] += j;
                        }
                    }
                }
            }
        }
    
    }
}
