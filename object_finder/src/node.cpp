#include <iostream>
#include <time.h>

#include <boost/foreach.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <uf_common/msg_helpers.h>

#include "sphere_finding.h"

using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace visualization_msgs;
using namespace Eigen;
using namespace uf_common;


const double buoy_r = 0.095;

double gauss() {
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(boost::mt19937(time(0)), boost::normal_distribution<>());
    return generator();
}


double uniform() {
    static boost::mt19937 m(time(0)-3);
    return boost::uniform_real<double>()(m);
}

struct Particle {
    btVector3 pos;
    Particle() {
        pos = btVector3(-2+gauss(), 2+gauss(), -1+gauss());
    }
    Particle(btVector3 pos) :
        pos(pos) {
    }
    Particle predict(double dt) {
        Particle p(*this);
        // diffusion to model INS's position drift and to test near solutions to try to find a better fit
        p.pos += btVector3(gauss(), gauss(), gauss()) * sqrt(.001 * dt);
        return p;
    }
    double P(const vector<Vector3d>& sumimage, const CameraInfoConstPtr& cam_info, const tf::StampedTransform& transform, vector<int>* dbg_image=NULL) {
        tf::Vector3 pos_camera_ = transform.inverse() * pos;
        Vector3d pos_camera(pos_camera_[0], pos_camera_[1], pos_camera_[2]);
        
        Vector3d inner_total_color; double inner_count;
        sphere_query(sumimage, cam_info, pos_camera, buoy_r, inner_total_color, inner_count, dbg_image);
        
        Vector3d both_total_color; double both_count;
        sphere_query(sumimage, cam_info, pos_camera, 2*buoy_r, both_total_color, both_count);
        
        Vector3d outer_total_color = both_total_color - inner_total_color;
        double outer_count = both_count - inner_count;
        
        //cout << "count: " << count << endl;
        if(inner_count > 10 && outer_count > 10) {
            Vector3d inner_color = inner_total_color/inner_count;
            Vector3d outer_color = outer_total_color/outer_count;
            
            return (inner_color - outer_color).norm();
            
            //cout << "color: " << color << endl;
            
            // assume that pixel color at object's center follows a triangular distribution with max at (0, 1, 0) and minimums at all other corners
            return 2*(  inner_color(0)) * 2*(1-inner_color(1)) * 2*(1-inner_color(2)) *
                   2*(1-outer_color(0)) * 2*(1-outer_color(1)) * 2*(  outer_color(2))
                + 1e-6; // 1e-6 is to make sure every particle doesn't go to 0 at the same time
        } else {
            // not visible
            return .3; // expected value of above expression
        }
    }
};

struct Node {
    ros::NodeHandle nh;
    tf::TransformListener listener;
    message_filters::Subscriber<Image> image_sub;
    message_filters::Subscriber<CameraInfo> info_sub;
    TimeSynchronizer<Image, CameraInfo> sync;
    ros::Publisher particles_pub;
    ros::Publisher pose_pub;
    ros::Publisher marker_pub;
    ros::Publisher image_pub;
    
    
    typedef std::pair<double, Particle> pair_type;
    std::vector<pair_type> particles;
    ros::Time current_stamp;
    
    
    Node() :
        image_sub(nh, "camera/image_rect_color", 1),
        info_sub(nh, "camera/camera_info", 1),
        sync(image_sub, info_sub, 10) {
        
        particles_pub = nh.advertise<Marker>("particles", 1);
        pose_pub = nh.advertise<PoseStamped>("buoy", 1);
        marker_pub = nh.advertise<Marker>("buoy_rviz", 1);
        image_pub = nh.advertise<Image>("camera/image_debug", 1);
        
        sync.registerCallback(boost::bind(&Node::callback, this, _1, _2));
    }
    
    void init_particles(ros::Time t) {
        particles.clear();
        
        for(int i = 0; i < 2000; i++)
            particles.push_back(make_pair(1, Particle()));
        
        double total_weight = 0; BOOST_FOREACH(pair_type &pair, particles) total_weight += pair.first;
        BOOST_FOREACH(pair_type &pair, particles) pair.first /= total_weight;
        
        current_stamp = t;
    }
    
    void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info) {
        assert(image->header.stamp == cam_info->header.stamp);
        assert(image->header.frame_id == cam_info->header.frame_id);
        
        if(particles.size() == 0) {
            ROS_INFO("got first frame; initializing particles");
            init_particles(image->header.stamp);
        } else if(image->header.stamp < current_stamp - ros::Duration(1) || image->header.stamp > current_stamp + ros::Duration(5)) {
            ROS_INFO("time jumped too far backwards or forwards; reinitializing particles");
            init_particles(image->header.stamp);
        } else if(image->header.stamp < current_stamp) {
            ROS_ERROR("dropped out of order camera frame");
            return;
        }
        double dt = (image->header.stamp - current_stamp).toSec();
        current_stamp = image->header.stamp;
        
        // get map from camera transform
        tf::StampedTransform transform;
        try {
            listener.setExtrapolationLimit(ros::Duration(0.3));
            listener.waitForTransform("/map", image->header.frame_id, image->header.stamp, ros::Duration(0.05));
            listener.lookupTransform("/map", image->header.frame_id, image->header.stamp, transform);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            return;
        }
        ROS_INFO("good");
        
        // decide whether resampling is necessary
        double weight2_sum = 0; BOOST_FOREACH(pair_type &pair, particles) weight2_sum += pair.first * pair.first;
        double N_eff = 1 / weight2_sum;
        if(N_eff < particles.size()*1/2) {
            // residual resampling
            std::vector<std::pair<double, Particle> > new_particles;
            BOOST_FOREACH(pair_type &pair, particles) {
                int count = particles.size() * pair.first;
                for(int i = 0; i < count; i++)
                    new_particles.push_back(std::make_pair(1./particles.size(), pair.second.predict(dt)));
                pair.first -= (double)count / particles.size();
            }

            vector<double> cumulative_weights(particles.size());
            cumulative_weights[0] = particles[0].first;
            for(unsigned int i = 1; i < particles.size(); i++)
                cumulative_weights[i] = cumulative_weights[i-1] + particles[i].first;
            while(new_particles.size() < particles.size()) {
                int k = lower_bound(cumulative_weights.begin(), cumulative_weights.end(), uniform()*cumulative_weights[particles.size()-1]) - cumulative_weights.begin();
                new_particles.push_back(std::make_pair(1./particles.size(), particles[k].second.predict(dt)));
            }

            particles = new_particles;
        } else {
            BOOST_FOREACH(pair_type &pair, particles) pair.second = pair.second.predict(dt);
        }
        
        static vector<Vector3d> sumimage; // static so not reallocated every frame
        sumimage.resize(image->width * image->height);
        
        int step;
        if(image->encoding == "rgba8") step = 4;
        else if(image->encoding == "rgb8") step = 3;
        else assert(false);
        for(uint32_t row = 0; row < image->height; row++) {
            Vector3d row_cumulative_sum(0, 0, 0);
            for(uint32_t col = 0; col < image->width; col++) {
                const uint8_t *pixel = image->data.data() + image->step * row + step * col;
                double r = pixel[0] / 255.;
                double g = pixel[1] / 255.;
                double b = pixel[2] / 255.;
                
                row_cumulative_sum += Vector3d(r, g, b);
                
                sumimage[image->width * row + col] = row_cumulative_sum;
            }
        }
        
        // update weights
        BOOST_FOREACH(pair_type &pair, particles) pair.first *= pair.second.P(sumimage, cam_info, transform);
        
        // normalize weights
        double total_weight = 0; BOOST_FOREACH(pair_type &pair, particles) total_weight += pair.first;
        BOOST_FOREACH(pair_type &pair, particles) pair.first /= total_weight;
        
        pair_type max_p; max_p.first = -1; BOOST_FOREACH(pair_type &pair, particles) if(pair.first > max_p.first) max_p = pair;
        
        btVector3 mean_position(0, 0, 0); BOOST_FOREACH(pair_type &pair, particles) mean_position += pair.first * pair.second.pos;
        
        // send marker message for particle visualization
        Marker msg;
        msg.header.frame_id = "/map";
        msg.header.stamp = image->header.stamp;
        msg.type = Marker::POINTS;
        msg.scale = make_xyz<Vector3>(buoy_r, buoy_r, 1);
        msg.color.a = 1;
        BOOST_FOREACH(pair_type &pair, particles) {
            msg.points.push_back(vec2xyz<Point>(pair.second.pos));
            msg.colors.push_back(make_rgba<ColorRGBA>(pair.first/max_p.first, 0, 1-pair.first/max_p.first, 1));
        }
        particles_pub.publish(msg);
        
        
        PoseStamped msg2;
        msg2.header.frame_id = "/map";
        msg2.header.stamp = image->header.stamp;
        msg2.pose.position = vec2xyz<Point>(max_p.second.pos);
        msg2.pose.orientation = make_xyzw<geometry_msgs::Quaternion>(0, 0, 0, 1);
        pose_pub.publish(msg2);
        
        Marker msg3;
        msg3.header.frame_id = "/map";
        msg3.header.stamp = image->header.stamp;
        msg3.pose.position = vec2xyz<Point>(mean_position);
        msg3.pose.orientation = make_xyzw<geometry_msgs::Quaternion>(0, 0, 0, 1);
        msg3.type = Marker::SPHERE;
        msg3.scale = make_xyz<Vector3>(2*buoy_r, 2*buoy_r, 2*buoy_r);
        msg3.color = make_rgba<ColorRGBA>(0, 1, 0, 1);
        marker_pub.publish(msg3);
        
        if(image_pub.getNumSubscribers()) {
            vector<int> dbg_image(image->width*image->height, 0);
            Particle(mean_position).P(sumimage, cam_info, transform, &dbg_image);
            
            Image msg4;
            msg4.header = image->header;
            msg4.height = image->height;
            msg4.width = image->width;
            msg4.encoding = "rgb8";
            msg4.is_bigendian = 0;
            msg4.step = image->width*3;
            msg4.data.resize(image->width*image->height*3);
            for(unsigned int y = 0; y < image->height; y++) {
                for(unsigned int x = 0; x < image->width; x++) {
                    msg4.data[msg4.step*y + 3*x + 0] = 100*dbg_image[image->width*y + x];
                    const uint8_t *pixel = image->data.data() + image->step * y + step * x;
                    msg4.data[msg4.step*y + 3*x + 1] = pixel[1];
                }
            }
            image_pub.publish(msg4);
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder");
    
    Node n;
    
    ros::spin();
    
    return 0;
}
