#include <boost/foreach.hpp>
#include <time.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>


#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace visualization_msgs;


static std_msgs::ColorRGBA make_ColorRGBA(float r, float g, float b, float a) {
    std_msgs::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

static geometry_msgs::Point make_Point(double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

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
        // prior distribution of object is centered at (3, 10, -4)
        pos = btVector3(3+gauss(), 10+gauss(), -4+gauss());
    }
    Particle predict(double dt) {
        Particle p(*this);
        // diffusion to model INS's position drift and to test near solutions to try to find a better fit
        p.pos += btVector3(gauss(), gauss(), gauss()) * sqrt(.001 * dt);
        return p;
    }
    double P(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info, const tf::StampedTransform& transform) {
        tf::Vector3 pos_camera = transform.inverse() * pos;
        
        if(pos_camera[2] <= 0) return 1; // behind camera
        
        double u = cam_info->P[ 0] * pos_camera[0] + cam_info->P[ 1] * pos_camera[1] + cam_info->P[ 2] * pos_camera[2] + cam_info->P[ 3] * 1;
        double v = cam_info->P[ 4] * pos_camera[0] + cam_info->P[ 5] * pos_camera[1] + cam_info->P[ 6] * pos_camera[2] + cam_info->P[ 7] * 1;
        double w = cam_info->P[ 8] * pos_camera[0] + cam_info->P[ 9] * pos_camera[1] + cam_info->P[10] * pos_camera[2] + cam_info->P[11] * 1;
        
        double x = u / w;
        double y = v / w;
        
        int xx = x + .5;
        int yy = y + .5;
        
        if(xx < 0 || xx >= (signed)image->width || yy < 0 || yy >= (signed)image->height)
            return 1; // not within camera's field of view
        else {
            assert(image->encoding == "rgba8"); // XXX use opencv to support more than one encoding
            const uint8_t *pixel = image->data.data() + image->step * yy + 4 * xx;
            double r = pixel[0] / 255.;
            double g = pixel[1] / 255.;
            double b = pixel[2] / 255.;
            
            // assume that pixel color at object's center follows a triangular distribution with max at (0, 1, 0) and minimums at all other corners
            // XXX this is where most of the work is to be done
            return 2*(1-r) * 2*(g) * 2*(1-b) + 1e-6; // 1e-6 is to make sure every particle doesn't go to 0 at the same time
        }
        
        return 1;
    }
};

struct Node {
    ros::NodeHandle nh;
    tf::TransformListener listener;
    ros::Publisher particles_pub;
    message_filters::Subscriber<Image> image_sub;
    message_filters::Subscriber<CameraInfo> info_sub;
    TimeSynchronizer<Image, CameraInfo> sync;
    
    
    typedef std::pair<double, Particle> pair_type;
    std::vector<pair_type> particles;
    ros::Time current_stamp;
    
    
    Node() :
        image_sub(nh, "image", 1),
        info_sub(nh, "camera_info", 1),
        sync(image_sub, info_sub, 10) {
        
        particles_pub = nh.advertise<Marker>("particles", 1);
        
        sync.registerCallback(boost::bind(&Node::callback, this, _1, _2));
        
        for(int i = 0; i < 100000; i++)
            particles.push_back(make_pair(1, Particle()));
        
        double total_weight = 0; BOOST_FOREACH(pair_type &pair, particles) total_weight += pair.first;
        BOOST_FOREACH(pair_type &pair, particles) pair.first /= total_weight;
        
        current_stamp = ros::Time::now();
    }
    
    void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info) {
        assert(image->header.stamp == cam_info->header.stamp);
        assert(image->header.frame_id == cam_info->header.frame_id);
        
        if(image->header.stamp < current_stamp) {
            ROS_ERROR("dropped out of order camera frame");
            return;
        }
        double dt = (image->header.stamp - current_stamp).toSec();
        current_stamp = image->header.stamp;
        
        // get map from camera transform
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", image->header.frame_id, image->header.stamp, transform);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            return;
        }
        
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
        
        // update weights
        BOOST_FOREACH(pair_type &pair, particles) pair.first *= pair.second.P(image, cam_info, transform);
        
        // normalize weights
        double total_weight = 0; BOOST_FOREACH(pair_type &pair, particles) total_weight += pair.first;
        BOOST_FOREACH(pair_type &pair, particles) pair.first /= total_weight;
        
        
        // send marker message for particle visualization
        Marker msg;
        msg.header.frame_id = "/map";
        msg.header.stamp = image->header.stamp;
        msg.type = Marker::POINTS;
        msg.scale.x = .4;
        msg.scale.y = .4;
        msg.scale.z = 1;
        msg.color.a = 1;
        double max_p = 0; BOOST_FOREACH(pair_type &pair, particles) if(pair.first > max_p) max_p = pair.first;
        BOOST_FOREACH(pair_type &pair, particles) {
            msg.points.push_back(make_Point(pair.second.pos.x(), pair.second.pos.y(), pair.second.pos.z()));
            msg.colors.push_back(make_ColorRGBA(pair.first/max_p, 0, 1-pair.first/max_p, 1));
        }
        particles_pub.publish(msg);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder");
    
    Node n;
    
    ros::spin();
    
    return 0;
}
