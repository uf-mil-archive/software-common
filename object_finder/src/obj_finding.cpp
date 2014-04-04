// triangles must not overlap when projected onto image, which essentially means
// that they should be planar

#include <sstream>
#include <fstream>

#include <boost/foreach.hpp>

#include "obj_finding.h"


using namespace Eigen;
using namespace sensor_msgs;

namespace object_finder {
namespace obj_finding {

static Vector3d flat_vector(Vector2d x) {
    return Vector3d(x[0], x[1], 0);
}

static Vector3d xyzvec(geometry_msgs::Point x) {
    return Vector3d(x.x, x.y, x.z);
}

double triangle_area(Eigen::Vector3d corner0, Eigen::Vector3d corner1, Eigen::Vector3d corner2) {
    Eigen::Vector3d a = corner1 - corner0;
    Eigen::Vector3d b = corner2 - corner0;
    return a.cross(b).norm()/2;
}

void draw(Component const & component, RenderBuffer &renderbuffer, int region, Vector3d pos, Quaterniond orientation) {
    BOOST_FOREACH(const Triangle &tri, component.triangles) {
        // XXX one corner being behind the camera does not mean the triangle is invisible!
        // instead, it is an "external triangle" that needs to be handled specially
        
        Vector3d c0_camera = renderbuffer.img->transform_inverse * (pos + orientation._transformVector(xyzvec(tri.corners[0])));
        Vector3d c0_homo = renderbuffer.img->proj * c0_camera.homogeneous();
        if(c0_homo(2) <= 0) continue; // behind camera
        Vector2d c0 = c0_homo.hnormalized();
        
        Vector3d c1_camera = renderbuffer.img->transform_inverse * (pos + orientation._transformVector(xyzvec(tri.corners[1])));
        Vector3d c1_homo = renderbuffer.img->proj * c1_camera.homogeneous();
        if(c1_homo(2) <= 0) continue; // behind camera
        Vector2d c1 = c1_homo.hnormalized();
        
        Vector3d c2_camera = renderbuffer.img->transform_inverse * (pos + orientation._transformVector(xyzvec(tri.corners[2])));
        Vector3d c2_homo = renderbuffer.img->proj * c2_camera.homogeneous();
        if(c2_homo(2) <= 0) continue; // behind camera
        Vector2d c2 = c2_homo.hnormalized();
        
        renderbuffer.areas[region] += triangle_area(flat_vector(c0), flat_vector(c1), flat_vector(c2));
        
        // sort corners by y coordinate
        if(c1[1] < c0[1]) std::swap(c1, c0);
        if(c2[1] < c1[1]) std::swap(c2, c1);
        if(c1[1] < c0[1]) std::swap(c1, c0);
        
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
            for(int Y = std::max(0, (int)ceil(ca[1] - 0.5)); Y + 0.5 < cb[1] && Y < (int)renderbuffer.img->height; Y++) {
                double y = Y + 0.5;
                double x1 = (y - c0[1])*(c2[0] - c0[0])/(c2[1] - c0[1]) + c0[0];
                double x2 = (y - ca[1])*(cb[0] - ca[0])/(cb[1] - ca[1]) + ca[0];
                if(x2 < x1) std::swap(x2, x1); // sort x's
                int X1 = std::max(renderbuffer.img->left[Y], std::min(renderbuffer.img->right[Y]-1, (int)ceil(x1 - 0.5)));
                int X2 = std::max(renderbuffer.img->left[Y], std::min(renderbuffer.img->right[Y]-1, (int)ceil(x2 - 0.5)));
                
                double z_0 = 0; // XXX
                double z_slope = 0; // XXX
                renderbuffer.scanlines[Y].add_segment(Segment(X1, X2, z_0, z_slope, region));
            }
        }
    }
}

}
}
