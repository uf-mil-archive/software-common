#include <iostream>
#include <time.h>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>

#include <uf_common/msg_helpers.h>

#include "object_finder/FindAction.h"
#include "sphere_finding.h"
#include "obj_finding.h"
#include "image.h"

using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace Eigen;
using namespace uf_common;


double gauss() {
    static boost::variate_generator<boost::mt19937,
        boost::normal_distribution<> > generator(boost::mt19937(time(0)),
            boost::normal_distribution<>());
    return generator();
}
Vector3d gaussvec() {
    return Vector3d(gauss(), gauss(), gauss());
}


double uniform() {
    static boost::mt19937 m(time(0)-3);
    return boost::uniform_real<double>()(m);
}

Affine3d eigen_from_tf(tf::Transform transform) {
    Affine3d x;
    x = Translation3d(vec2vec(transform.getOrigin()));
    tf::Quaternion q = transform.getRotation();
    x *= Quaterniond(q.w(), q.x(), q.y(), q.z());
    return x;
}

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

Quaterniond quat_from_rotvec(Vector3d r) {
    if(r.norm() == 0) return Quaterniond::Identity();
    return Quaterniond(AngleAxisd(r.norm(), r.normalized()));
}

struct Particle {
    static const double P_no_observation = 1;
    
    object_finder::FindGoal goal;
    boost::shared_ptr<const Obj> obj;
    Vector3d pos;
    Quaterniond q;
    Particle() { }
    Particle(const object_finder::FindGoal &goal,
             boost::shared_ptr<const Obj> obj,
             const TaggedImage &image) :
        goal(goal),
        obj(obj) {
        Matrix6d cov = Map<const Matrix6d>(
            goal.prior_distribution.covariance.data());
        Matrix6d dist_from_iid = cov.llt().matrixL();
        Vector6d iid; iid << gaussvec(), gaussvec();
        Vector6d dx = dist_from_iid * iid;
        
        pos = dx.head(3) + xyz2vec(goal.prior_distribution.pose.position);
        if(goal.max_dist) {
            pos = image.get_pixel_point(
                Vector2d(
                    uniform()*image.cam_info.width,
                    uniform()*image.cam_info.height),
                uniform()*(goal.max_dist-goal.min_dist)+goal.min_dist);
        }
        q = quat_from_rotvec(dx.tail(3)) *
            xyzw2quat(goal.prior_distribution.pose.orientation);
    }
    Particle(const object_finder::FindGoal &goal,
             boost::shared_ptr<const Obj> obj,
             Vector3d pos, Quaterniond q) :
        goal(goal), obj(obj),
        pos(pos), q(q) {
    }
    Particle predict(double dt) {
        Particle p(*this);
        // diffusion to model INS's position drift and to test near solutions 
        // to try to find a better fit
        p.pos += sqrt(.01 * dt) * gaussvec();
        //p.q *= quat_from_rotvec(sqrt(.01 * dt) * gaussvec());
        p.q = quat_from_rotvec(Vector3d(0, 0, sqrt(.01*dt) * gauss())) * p.q;
        if(goal.allow_pitching) {
            p.q = p.q * quat_from_rotvec(
                Vector3d(0, sqrt(.01 * dt) * gauss(), 0));
        }
        return p;
    }
    double P(const TaggedImage &img, vector<int>* dbg_image=NULL) {
        Result inner_result;
        Result outer_result;
        vector<ResultWithArea> results;
        if(goal.type == object_finder::FindGoal::TYPE_SPHERE) {
            sphere_query(img, pos, goal.sphere_radius,
                inner_result, dbg_image);
            
            Result both_result;
            sphere_query(img, pos, 2*goal.sphere_radius,
                both_result, dbg_image);
            outer_result = both_result - inner_result;
        } else if(goal.type == object_finder::FindGoal::TYPE_OBJECT) {
            inner_result = Result::Zero();
            outer_result = Result::Zero();
            
            obj->query(img, pos, q, results, dbg_image);
            
            for(unsigned int i = 0; i < results.size(); i++) {
                if(obj->components[i].name.find("solid_") == 0) {
                    inner_result += results[i];
                } else if(obj->components[i].name.find("background_") == 0) {
                    outer_result += results[i];
                }
                if(dbg_image) {
                    cout << obj->components[i].name
                        << " count: " << results[i].count << " "
                        << results[i].total_color
                        << "area: " << results[i].area << endl;
                }
            }
        } else {
            cout << "Invalid type:" << goal.type << endl;
            assert(false);
        }
        
        Result far_result = img.total_result - inner_result; //- outer_result;
        
        if(inner_result.count < 100 || outer_result.count < 100) {
            return 0.001;
            if(dbg_image) {
                cout << endl;
                cout << endl;
                cout << "NOT VISIBLE" << endl;
            }
            // not visible
            return P_no_observation; // expected value of above expression
        }
        
        Vector3d inner_color = inner_result.avg_color().normalized();
        Vector3d outer_color = outer_result.avg_color().normalized();
        Vector3d far_color = far_result.avg_color().normalized();
        //double P2 = (inner_color - outer_color).norm();
        //double P = P2* pow(far_color.dot(outer_color), 5);
        double P = 1;
        for(unsigned int i = 0; i < results.size(); i++) {
            if(obj->components[i].name.find("solid_") != 0 &&
               obj->components[i].name.find("background_") != 0) {
                continue;
            }
            if(results[i].count < 100) {
                if(dbg_image) {
                    cout << "TOO SMALL" << obj->components[i].name << endl;
                }
                P *= 0.5;
                continue;
            }
            if(obj->components[i].name.find("solid_") == 0) {
                Vector3d this_color = results[i].avg_color_assuming_unseen_is(
                    outer_color).normalized();
                P *= exp(10*(
                    (this_color - outer_color).norm() -
                    ( far_color - outer_color).norm() - .05
                ));
            } else if(obj->components[i].name.find("background_") == 0) {
                Vector3d this_color = results[i].avg_color_assuming_unseen_is(
                    inner_color).normalized();
                P *= exp(10*(
                    (inner_color - this_color).norm() -
                    (  far_color - this_color).norm() - .05
                ));
            }
        }
        
        if(goal.type == object_finder::FindGoal::TYPE_SPHERE) {
            P = exp(10*(
                (inner_color - outer_color).norm() -
                (  far_color - outer_color).norm() - .01
            ));
        }
        
        if(dbg_image) {
            cout << endl;
            cout << endl;
            cout << "inner count" << inner_result.count << endl;
            cout << "inner avg " << inner_color << endl;
            cout << endl;
            cout << "outer count" << outer_result.count << endl;
            cout << "outer avg " << outer_color << endl;
            cout << endl;
            cout << "far count" << far_result.count << endl;
            cout << "far avg " << far_color << endl;
            cout << endl;
            cout << "P " << P << endl;
        }
        if(!(isfinite(P) && P >= 0)) {
            cout << "bad P: " << P << endl;
            assert(false);
        }
        return P;
    }
};

struct Node {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    actionlib::SimpleActionServer<object_finder::FindAction> actionserver;
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<CameraInfo> info_sub;
    tf::MessageFilter<CameraInfo> info_tf_filter;
    TimeSynchronizer<Image, CameraInfo> sync;
    ros::Publisher particles_pub;
    ros::Publisher pose_pub;
    ros::Publisher image_pub;
    
    bool have_goal;
    object_finder::FindGoal goal;
    
    boost::shared_ptr<Obj> current_obj;
    
    typedef std::pair<double, Particle> pair_type;
    std::vector<pair_type> particles;
    ros::Time current_stamp;
    double infinite_p;
    

    static bool compare_weight(const pair_type a, const pair_type b) {
        return a.first < b.first;
    }
    
    Node() :
        private_nh("~"),
        actionserver(nh, "find", false),
        image_sub(nh, "camera/image_rect_color", 1),
        info_sub(nh, "camera/camera_info", 1),
        info_tf_filter(info_sub, listener, "", 10),
        sync(image_sub, info_tf_filter, 10),
        have_goal(false) {
        
        particles_pub = private_nh.advertise<visualization_msgs::Marker>(
            "particles", 1);
        pose_pub = private_nh.advertise<PoseStamped>("pose", 1);
        image_pub = private_nh.advertise<sensor_msgs::Image>(
            "debug_image", 1);
        
        sync.registerCallback(boost::bind(&Node::callback, this, _1, _2));
        
        actionserver.registerGoalCallback(
            boost::bind(&Node::goalCallback, this));
        actionserver.start();
    }
    
    void init_particles(ros::Time t, const TaggedImage &img) {
        particles.clear();
        
        int N = 1000;
        
        for(int i = 0; i < N; i++)
            particles.push_back(
                make_pair(1./N, Particle(goal, current_obj, img)));
        
        infinite_p = 1./N;
        
        current_stamp = t;
    }
    
    void goalCallback() {
        have_goal = true;
        boost::shared_ptr<const object_finder::FindGoal> new_goal =
            actionserver.acceptNewGoal();
        goal = *new_goal;
        particles.clear();
        current_obj.reset();
        if(goal.type == object_finder::FindGoal::TYPE_OBJECT) {
            current_obj = boost::make_shared<Obj>(goal.object_filename);
        }
        info_tf_filter.setTargetFrame(goal.header.frame_id);
    }
    
    void callback(const ImageConstPtr& image,
                  const CameraInfoConstPtr& cam_info) {
        assert(image->header.stamp == cam_info->header.stamp);
        assert(image->header.frame_id == cam_info->header.frame_id);
        
        if(actionserver.isPreemptRequested()) {
            have_goal = false;
            current_obj.reset();
        }
        
        if(!have_goal)
            return;
        
        double dt = (image->header.stamp - current_stamp).toSec();
        current_stamp = image->header.stamp;
        
        // get map from camera transform
        tf::StampedTransform transform;
        try {
            listener.lookupTransform(goal.header.frame_id,
                image->header.frame_id, image->header.stamp, transform);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            return;
        }
        ROS_INFO("good");
        
        TaggedImage img(*image, *cam_info, eigen_from_tf(transform));
        
        if(particles.size() == 0) {
            ROS_INFO("got first frame; initializing particles");
            init_particles(image->header.stamp, img);
        } else if(image->header.stamp < current_stamp - ros::Duration(1) ||
                  image->header.stamp > current_stamp + ros::Duration(5)) {
            ROS_INFO("time jumped too far backwards or forwards; "
                "reinitializing particles");
            init_particles(image->header.stamp, img);
        } else if(image->header.stamp < current_stamp) {
            ROS_ERROR("dropped out of order camera frame");
            return;
        }
        
        // decide whether resampling is necessary
        double weight2_sum = 0;
        BOOST_FOREACH(pair_type &pair, particles)
            weight2_sum += pair.first * pair.first;
        
        double N_eff = 1 / weight2_sum;
        if(N_eff < particles.size()*1/2 || true) {
            // residual resampling
            std::vector<std::pair<double, Particle> > new_particles;
            BOOST_FOREACH(pair_type &pair, particles) {
                int count = particles.size() * pair.first;
                for(int i = 0; i < count; i++)
                    new_particles.push_back(
                        std::make_pair(1./particles.size(), i==0?
                            pair.second :
                            pair.second.predict(dt)));
                pair.first -= (double)count / particles.size();
            }

            vector<double> cumulative_weights(particles.size());
            cumulative_weights[0] = particles[0].first;
            for(unsigned int i = 1; i < particles.size(); i++)
                cumulative_weights[i] = cumulative_weights[i-1] +
                    particles[i].first;
            while(new_particles.size() < particles.size()) {
                int k = lower_bound(cumulative_weights.begin(),
                    cumulative_weights.end(),
                    uniform()*cumulative_weights[particles.size()-1]) -
                    cumulative_weights.begin();
                new_particles.push_back(
                    std::make_pair(1./particles.size(),
                        particles[k].second.predict(dt)));
            }

            particles = new_particles;
        } else {
            BOOST_FOREACH(pair_type &pair, particles)
                pair.second = pair.second.predict(dt);
        }
        
        // update weights
        BOOST_FOREACH(pair_type &pair, particles)
            pair.first *= pair.second.P(img);
        infinite_p *= Particle::P_no_observation * 1.0001;
        
        sort(particles.begin(), particles.end(), compare_weight);
        for(unsigned int i = 0; i < particles.size()/2; i++) {
            pair_type &pair = particles[i];
            if(pair.first < infinite_p) {
                pair.first = infinite_p;
                pair.second = Particle(goal, current_obj, img);
            }
        }
        
        // normalize weights
        double total_weight = 0;
        BOOST_FOREACH(pair_type &pair, particles) total_weight += pair.first;
        
        BOOST_FOREACH(pair_type &pair, particles) pair.first /= total_weight;
        infinite_p /= total_weight;
        
        pair_type max_p; max_p.first = -1;
        BOOST_FOREACH(pair_type &pair, particles)
            if(pair.first > max_p.first) max_p = pair;
        cout << "max_p.first: " << max_p.first << endl;
        cout << "particles.size(): " << particles.size() << endl;
        cout << "particles[0].first: " << particles[0].first << endl;
        assert(max_p.first >= 0);
        
        Vector3d mean_position(0, 0, 0);
        Vector4d q_sum(0, 0, 0, 0);
        BOOST_FOREACH(const pair_type &pair, particles) {
            mean_position += pair.first * pair.second.pos;
            
            vector<Vector4d> qs;
            qs.push_back( pair.second.q.coeffs());
            qs.push_back(-pair.second.q.coeffs());
            if(goal.is_180z_symmetric) {
                qs.push_back( (pair.second.q*Quaterniond(0,0,1,0)).coeffs());
                qs.push_back(-(pair.second.q*Quaterniond(0,0,1,0)).coeffs());
            }
            
            Vector4d best_q;
            double best_score = -1e300;
            BOOST_FOREACH(const Vector4d &q, qs) {
                double score = q.dot(q_sum);
                if(score > best_score) {
                    best_q = q;
                    best_score = score;
                }
            }
            q_sum += pair.first * best_q;
        }
        Quaterniond mean_orientation = Quaterniond(q_sum.normalized());
        
        // send marker message for particle visualization
        visualization_msgs::Marker msg;
        msg.header.frame_id = goal.header.frame_id;
        msg.header.stamp = image->header.stamp;
        msg.type = visualization_msgs::Marker::POINTS;
        msg.scale = make_xyz<Vector3>(.1, max(goal.sphere_radius, .1), 1);
        msg.color.a = 1;
        BOOST_FOREACH(pair_type &pair, particles) {
            msg.points.push_back(vec2xyz<Point>(pair.second.pos));
            msg.colors.push_back(make_rgba<ColorRGBA>(
                pair.first/max_p.first, 0, 1-pair.first/max_p.first, 1));
        }
        particles_pub.publish(msg);
        
        PoseStamped msg2;
        msg2.header.frame_id = goal.header.frame_id;
        msg2.header.stamp = image->header.stamp;
        msg2.pose.position = vec2xyz<Point>(mean_position);
        msg2.pose.orientation = quat2xyzw<geometry_msgs::Quaternion>(
            mean_orientation);
        pose_pub.publish(msg2);
        
        Particle mean_particle(goal, current_obj,
            mean_position, mean_orientation);
        mean_particle = max_p.second;
        
        object_finder::FindFeedback feedback;
        feedback.pose.position = vec2xyz<Point>(mean_position);
        feedback.pose.orientation = quat2xyzw<geometry_msgs::Quaternion>(
            mean_particle.q);
        feedback.P = mean_particle.P(img);
        feedback.P_within_10cm = 0;
        BOOST_FOREACH(pair_type &pair, particles)
            if((pair.second.pos - mean_particle.pos).norm() <= .2)
                feedback.P_within_10cm += pair.first;
        if(current_obj) {
            BOOST_FOREACH(const Marker &marker, current_obj->markers) {
                feedback.markers.push_back(object_finder::MarkerPoint());
                feedback.markers[feedback.markers.size()-1].name =
                    marker.name;
                feedback.markers[feedback.markers.size()-1].position =
                    vec2xyz<Point>(marker.position);
            }
        }
        actionserver.publishFeedback(feedback);
        
        if(image_pub.getNumSubscribers()) {
            vector<int> dbg_image(image->width*image->height, 0);
            //mean_particle.P(img, &dbg_image);
            max_p.second.P(img, &dbg_image);
            
            Image msg4;
            msg4.header = image->header;
            msg4.height = image->height;
            msg4.width = image->width;
            msg4.encoding = "rgb8";
            msg4.is_bigendian = 0;
            msg4.step = image->width*3;
            msg4.data.resize(image->width*image->height*3);
            int step;
            if(image->encoding == "rgba8" || image->encoding == "bgra8")
                step = 4;
            else if(image->encoding == "rgb8" || image->encoding == "bgr8")
                step = 3;
            else assert(false);
            for(unsigned int y = 0; y < image->height; y++) {
                for(unsigned int x = 0; x < image->width; x++) {
                    msg4.data[msg4.step*y + 3*x + 0] =
                        100*dbg_image[image->width*y + x];
                    const uint8_t *pixel = image->data.data() +
                        image->step * y + step * x;
                    msg4.data[msg4.step*y + 3*x + 1] = pixel[1];
                    msg4.data[msg4.step*y + 3*x + 2] = pixel[2];
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
