#include <fstream>

#include <boost/foreach.hpp>

#include "object_finder/Mesh.h"
#include "obj_finding.h"
#include "sphere_finding.h"

using namespace std;
using namespace Eigen;

void write(const vector<int> &dbg_image, int width, int height, string filename) {
    ofstream f(filename.c_str());
    int max_val = 0; for(int i = 0; i < width*height; i++) max_val = max(max_val, dbg_image[i]);
    f << "P2 " << width << " " << height << " " << max_val << "\n";
    for(int Y = 0; Y < height; Y++) {
        for(int X = 0; X < width; X++) {
            f << dbg_image[Y * width + X] << " ";
        }
        f << "\n";
    }
    f.close();
    cout << "Wrote to " << filename << endl;
}

object_finder::Mesh load_mesh(string const & filename) {
    vector<geometry_msgs::Point> vertices;
    vector<object_finder::Component> components;
    
    ifstream f(filename.c_str());
    if(!f.is_open()) {
        throw runtime_error("couldn't open file");
    }
    
    bool ignore_faces = true;
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
            string name; ss >> name;
            replace(name.begin(), name.end(), '_', ' ');
            if(name.find("ignore ") != 0) {
                components.push_back(object_finder::Component());
                components[components.size()-1].name = name;
                ignore_faces = false;
            } else {
                ignore_faces = true;
            }
        } else if(command == "v") {
            double x, y, z; ss >> x >> y >> z;
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            vertices.push_back(point);
        } else if(command == "f") {
            int i0, i1, i2; ss >> i0 >> i1 >> i2;
            object_finder::Triangle tri;
            tri.corners[0] = vertices[i0 - 1];
            tri.corners[1] = vertices[i1 - 1];
            tri.corners[2] = vertices[i2 - 1];
            if(!ignore_faces) {
                components[components.size()-1].triangles.push_back(tri);
            }
        }
    }
    sort(components.begin(), components.end(),
        [](object_finder::Component const &a,
           object_finder::Component const &b) {
            return a.name < b.name;
        }); // sort by name
    
    object_finder::Mesh res;
    res.components = components;
    return res;
}


int main() {
    int width=640, height=480;
    sensor_msgs::CameraInfo camera_info;
    sensor_msgs::Image image;
    image.width = camera_info.width = width;
    image.height = camera_info.height = height;
    image.encoding = "rgb8";
    image.data.resize(3 * width * height);
    
    double tmp[12] = {167.819926235456, 0.0, 349.5, 0.0, 0.0, 167.819926235456, 199.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    for(int i = 0; i < 12; i++) camera_info.P[i] = tmp[i];
    
    TaggedImage img(image, camera_info, Affine3d::Identity());
    
    //write(dbg_image, width, height, "tmp.pgm");
    
    RenderBuffer rb(img);
    
    object_finder::Mesh mesh = load_mesh("shooter.obj");
    BOOST_FOREACH(const object_finder::Component &component, mesh.components) {
        object_finder::obj_finding::draw(component, rb, rb.new_region(), Vector3d(-1.5, 0, 2), Quaterniond(.5, .5, -.5, .5));
    }
    sphere_draw(rb, rb.new_region(), Vector3d(0, 0, 2), .3);
    sphere_draw(rb, rb.new_region(), Vector3d(1.5, 0, 2), .2);
    sphere_draw(rb, rb.new_region(), Vector3d(1.5, 0, 2), .3);
    
    vector<int> dbg_image(width*height, 0);
    rb.draw_debug_regions(dbg_image);
    
    vector<ResultWithArea> results = rb.get_results();
    
    BOOST_FOREACH(const Result &result, results) {
        cout << "region area: " << result.count << endl;
    }
    
    write(dbg_image, width, height, "out.pgm");
}
