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

using namespace object_finder;


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
    TargetDesc goal;
    boost::shared_ptr<const Obj> obj;
    Vector3d pos;
    Quaterniond q;
    RenderBuffer::RegionType bg_region;
    RenderBuffer::RegionType fg_region;
    
    Vector3d last_color;
    double last_P;
    double smoothed_last_P;
    
    Particle() { }
    Particle(const TargetDesc &goal,
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
        if(goal.check_180z_flip && uniform() >= .5)
            q = q * Quaterniond(0, 0, 1, 0);
        smoothed_last_P = 1;
    }
    Particle(const TargetDesc &goal,
             boost::shared_ptr<const Obj> obj,
             Vector3d pos, Quaterniond q) :
        goal(goal), obj(obj),
        pos(pos), q(q) {
        assert(false);
    }
    Particle realpredict(double amount) const {
        Particle p(*this);
        p.smoothed_last_P = 1.;
        // diffusion to test near solutions to try to find a better fit
        p.pos += amount * gaussvec();
        if(false) { // allow_rolling
            p.q *= quat_from_rotvec(amount * gaussvec());
        } else {
            p.q = quat_from_rotvec(Vector3d(0, 0, amount * gauss())) * p.q;
            if(goal.allow_pitching) {
                p.q = p.q * quat_from_rotvec(
                    Vector3d(0, amount * gauss(), 0));
            }
        }
        return p;
    }
    Particle predict() const {
        if(uniform() < .5) {
            return realpredict(.01);
        } else {
            return realpredict(.1);
        }
    }
    void draw1(RenderBuffer &renderbuffer) {
        bg_region = renderbuffer.new_region();
        if(!obj) {
            sphere_draw(renderbuffer, bg_region, pos, 2*goal.sphere_radius);
            return;
        }
        BOOST_FOREACH(const Component &component, obj->components) {
            if(component.name.find("background_") == 0) {
                component.draw(renderbuffer, bg_region, pos, q);
            }
        }
    }
    void draw2(RenderBuffer &renderbuffer) {
        fg_region = renderbuffer.new_region();
        if(!obj) {
            sphere_draw(renderbuffer, fg_region, pos, goal.sphere_radius);
            return;
        }
        BOOST_FOREACH(const Component &component, obj->components) {
            if(component.name.find("solid_") == 0) {
                component.draw(renderbuffer, fg_region, pos, q);
            }
        }
    }
    double P(const TaggedImage &img, RenderBuffer &rb, bool print_debug_info=false) {
        draw2(rb);
        draw1(rb);
        vector<ResultWithArea> results = rb.get_results();
        Result inner_result = results[fg_region];
        Result outer_result = results[bg_region];
        Result far_result = img.total_result - inner_result; //- outer_result;
        
        if(inner_result.count < 100 || outer_result.count < 100 || far_result.count < 100) {
            if(print_debug_info) {
                cout << endl;
                cout << endl;
                cout << "NOT VISIBLE" << endl;
            }
            // not visible
            return 0.001;
        }
        
        last_color = inner_result.avg_color(); // XXX
        
        Vector3d inner_color = inner_result.avg_color().normalized();
        Vector3d outer_color = outer_result.avg_color().normalized();
        Vector3d far_color = far_result.avg_color().normalized();
        //double P2 = (inner_color - outer_color).norm();
        //double P = P2* pow(far_color.dot(outer_color), 5);
        double P = 1;
        {
            if(results[fg_region].count < 100) {
                if(print_debug_info) {
                    cout << "TOO SMALL FG" << endl;
                }
                P *= 0.5;
            } else {
                Vector3d this_color = results[fg_region].avg_color_assuming_unseen_is(
                    outer_color).normalized();
                P *= exp(10*(
                    (this_color - outer_color).norm() -
                    ( far_color - outer_color).norm() - .05
                ));
            }
        }
        {
            if(results[bg_region].count < 100) {
                if(print_debug_info) {
                    cout << "TOO SMALL BG" << endl;
                }
                P *= 0.5;
            } else {
                Vector3d this_color = results[bg_region].avg_color_assuming_unseen_is(
                    inner_color).normalized();
                P *= exp(10*(
                    (inner_color - this_color).norm() -
                    (  far_color - this_color).norm() - .05
                ));
            }
        }
        
        if(print_debug_info) {
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
    void update(const TaggedImage &img, RenderBuffer &rb, bool print_debug_info=false) {
        double P = this->P(img, rb, print_debug_info);
        
        last_P = P;
        
        smoothed_last_P = pow(smoothed_last_P, 0.9) * pow(last_P, 0.1);
    }
    void accumulate_successors(std::vector<Particle> &res, const TaggedImage &img, double N, const std::vector<Particle> &particles, double total_last_P) const {
        int count = N * last_P/total_last_P + uniform();
        for(int i = 0; i < count; i++) {
            res.push_back(i==0 ?
                *this :
                predict());
        }
    }
    double dist(const Particle &other) {
        return (pos - other.pos).norm();
    }
};

struct ParticleFilter {
    TargetDesc targetdesc;
    boost::shared_ptr<const Obj> obj;
    std::vector<Particle> particles;
    double total_last_P;
    
    ParticleFilter(const TargetDesc &targetdesc, boost::shared_ptr<const Obj> obj, const TaggedImage &img, double N) :
        targetdesc(targetdesc),
        obj(obj) {
        for(int i = 0; i < N; i++)
            particles.push_back(Particle(targetdesc, obj, img));
    }
    
    RenderBuffer update(const TaggedImage &img, const RenderBuffer &rb, double N) {
        double original_total_last_P = 0;
        BOOST_FOREACH(const Particle &particle, particles) original_total_last_P += particle.last_P;
        
        // residual resampling
        std::vector<Particle> new_particles;
        BOOST_FOREACH(const Particle &particle, particles) {
            particle.accumulate_successors(new_particles, img, N*2/3., particles, original_total_last_P);
        }
        for(unsigned int i = 0; i < N/3; i++) {
            new_particles.push_back(Particle(targetdesc, obj, img));
        }
        particles.swap(new_particles);
        
        // update particles
        BOOST_FOREACH(Particle &particle, particles) {
            RenderBuffer tmp_rb = rb;
            particle.update(img, tmp_rb);
        }
        
        total_last_P = 0;
        BOOST_FOREACH(Particle &particle, particles) total_last_P += particle.last_P;
        
        RenderBuffer res = rb;
        get_best().P(img, res);
        return res;
    }
    
    Particle get_best() const {
        // find mode
        Particle max_p; max_p.smoothed_last_P = -1;
        BOOST_FOREACH(const Particle &particle, particles)
            if(particle.smoothed_last_P > max_p.smoothed_last_P) max_p = particle;
        assert(max_p.smoothed_last_P >= 0);
        return max_p;
    }
};

struct GoalExecutor {
    ros::NodeHandle nh;
    ros::NodeHandle camera_nh;
    ros::NodeHandle private_nh;
    object_finder::FindGoal goal;
    
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<CameraInfo> info_sub;
    tf::MessageFilter<CameraInfo> info_tf_filter;
    TimeSynchronizer<Image, CameraInfo> sync;
    ros::Publisher particles_pub;
    ros::Publisher pose_pub;
    ros::Publisher image_pub;
    boost::function1<void, const object_finder::FindFeedback&> feedback_callback;
    
    vector<boost::shared_ptr<Obj> > current_objs;
    
    double N;
    std::vector<ParticleFilter> particle_filters;
    ros::Time current_stamp;
    
    GoalExecutor(const object_finder::FindGoal &goal,
                 boost::function1<void, const object_finder::FindFeedback&> 
                    feedback_callback) :
        camera_nh("camera"),
        private_nh("~"),
        goal(goal),
        image_sub(camera_nh, "image_rect_color", 1),
        info_sub(camera_nh, "camera_info", 1),
        info_tf_filter(info_sub, listener, goal.header.frame_id, 10),
        sync(image_sub, info_tf_filter, 10),
        feedback_callback(feedback_callback) {
        
        BOOST_FOREACH(const TargetDesc &targetdesc, goal.targetdescs) {
            current_objs.push_back(targetdesc.type == TargetDesc::TYPE_OBJECT
                ? boost::make_shared<Obj>(targetdesc.object_filename)
                : boost::shared_ptr<Obj>());
        }
        
        particles_pub = private_nh.advertise<visualization_msgs::Marker>(
            "particles", 1);
        pose_pub = private_nh.advertise<PoseStamped>("pose", 1);
        image_pub = private_nh.advertise<sensor_msgs::Image>(
            "debug_image", 1);
        
        sync.registerCallback(boost::bind(&GoalExecutor::callback, this, _1, _2));
    }
    
    void init(ros::Time t, const TaggedImage &img) {
        N = 300;
        
        particle_filters.clear();
        BOOST_FOREACH(const TargetDesc &targetdesc, goal.targetdescs) {
            particle_filters.push_back(ParticleFilter(targetdesc,
                current_objs[&targetdesc - goal.targetdescs.data()], img, N));
        }
        
        current_stamp = t;
    }
    
    void callback(const ImageConstPtr& image,
                  const CameraInfoConstPtr& cam_info) {
        assert(image->header.stamp == cam_info->header.stamp);
        assert(image->header.frame_id == cam_info->header.frame_id);
        
        //double dt = (image->header.stamp - current_stamp).toSec();
        current_stamp = image->header.stamp;
        
        // get map from camera transform
        tf::StampedTransform transform;
        try {
            listener.lookupTransform(goal.header.frame_id,
                image->header.frame_id, image->header.stamp, transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        ROS_INFO("good");
        
        TaggedImage img(*image, *cam_info, eigen_from_tf(transform));
        
        if(particle_filters.size() == 0) {
            ROS_INFO("got first frame; initializing");
            init(image->header.stamp, img);
        } else if(image->header.stamp < current_stamp - ros::Duration(1) ||
                  image->header.stamp > current_stamp + ros::Duration(5)) {
            ROS_INFO("time jumped too far backwards or forwards; "
                "reinitializing");
            init(image->header.stamp, img);
        } else if(image->header.stamp < current_stamp) {
            ROS_ERROR("dropped out of order camera frame");
            return;
        }
        
        ros::WallTime start_time = ros::WallTime::now();
        
        //double total_smoothed_last_P = 0;
        //BOOST_FOREACH(Particle &particle, particles) total_smoothed_last_P += particle.smoothed_last_P;
        
        {
            RenderBuffer rb(img);
            BOOST_FOREACH(ParticleFilter &particle_filter, particle_filters) {
                rb = particle_filter.update(img, rb, N);
            }
        }
        
        std::vector<Particle> max_ps;
        BOOST_FOREACH(const ParticleFilter &particle_filter, particle_filters) {
            max_ps.push_back(particle_filter.get_best());
        }
        
        { // send marker message for particle visualization
            visualization_msgs::Marker msg;
            msg.header.frame_id = goal.header.frame_id;
            msg.header.stamp = image->header.stamp;
            msg.type = visualization_msgs::Marker::POINTS;
            msg.scale = make_xyz<Vector3>(.1, .1, 1);
            msg.color.a = 1;
            int i = 0; BOOST_FOREACH(const ParticleFilter &particle_filter, particle_filters) {
                BOOST_FOREACH(const Particle &particle, particle_filter.particles) {
                    msg.points.push_back(vec2xyz<Point>(particle.pos));
                    msg.colors.push_back(make_rgba<ColorRGBA>(
                        particle.smoothed_last_P/max_ps[i].smoothed_last_P, 0, 1-particle.smoothed_last_P/max_ps[i].smoothed_last_P, 1));
                }
             i++; }
            particles_pub.publish(msg);
        }
        
        { // send pose message for result visualization
            PoseStamped msg;
            msg.header.frame_id = goal.header.frame_id;
            msg.header.stamp = image->header.stamp;
            msg.pose.position = vec2xyz<Point>(max_ps[0].pos);
            msg.pose.orientation = quat2xyzw<geometry_msgs::Quaternion>(
                max_ps[0].q);
            pose_pub.publish(msg);
        }
        
        { // send action feedback
            object_finder::FindFeedback feedback;
            BOOST_FOREACH(const Particle &particle, max_ps) {
                TargetRes targetres;
                targetres.pose.position = vec2xyz<Point>(particle.pos);
                targetres.pose.orientation =
                     quat2xyzw<geometry_msgs::Quaternion>(particle.q);
                targetres.color = make_rgba<ColorRGBA>(particle.last_color[0],
                    particle.last_color[1], particle.last_color[2], 1);
                if(particle.obj) {
                    BOOST_FOREACH(const Marker &marker, particle.obj->markers) {
                        targetres.markers.push_back(object_finder::MarkerPoint());
                        targetres.markers[targetres.markers.size()-1].name =
                            marker.name;
                        targetres.markers[targetres.markers.size()-1].position =
                            vec2xyz<Point>(marker.position);
                    }
                }
                targetres.P = particle.last_P;
                targetres.smoothed_last_P = particle.smoothed_last_P;
                targetres.P_within_10cm = 0;
                ParticleFilter &particle_filter = particle_filters[&particle-max_ps.data()];
                BOOST_FOREACH(Particle &particle2, particle_filter.particles) {
                    if(particle2.dist(particle) <= .2)
                        targetres.P_within_10cm += particle2.last_P/particle_filter.total_last_P;
                }
                feedback.targetreses.push_back(targetres);
            }
            //RenderBuffer rb(img);
            // XXX
            //feedback.P = max_p.P(img, rb);
            //feedback.P_within_10cm = 0;
            //BOOST_FOREACH(Particle &particle, particles)
            //    if(particle.dist(max_p) <= .2)
            //        feedback.P_within_10cm += particle.smoothed_last_P/total_smoothed_last_P;
            feedback_callback(feedback);
        }
        
        ros::WallTime end_time = ros::WallTime::now();
        if(end_time - start_time > ros::WallDuration(.1)) {
            N *= .9;
        } else {
            N /= .9;
        }
        cout << "N = " << N << endl;
        
        if(image_pub.getNumSubscribers()) { // send debug image
            RenderBuffer rb(img);
            BOOST_FOREACH(Particle &max_p, max_ps) { // XXX make const
                max_p.P(img, rb, true);
            }
            
            vector<int> dbg_image(image->width*image->height, 0);
            rb.draw_debug_regions(dbg_image);
            
            Image msg;
            msg.header = image->header;
            msg.height = image->height;
            msg.width = image->width;
            msg.encoding = "rgb8";
            msg.is_bigendian = 0;
            msg.step = image->width*3;
            msg.data.resize(image->width*image->height*3);
            for(unsigned int y = 0; y < image->height; y++) {
                for(unsigned int x = 0; x < image->width; x++) {
                    Vector3d orig_color = img.get_pixel(y, x);
                    msg.data[msg.step*y + 3*x + 0] =
                        255.*dbg_image[image->width*y + x]/(rb.areas.size() + 1);
                    msg.data[msg.step*y + 3*x + 1] = 255*orig_color[1];
                    msg.data[msg.step*y + 3*x + 2] = 255*orig_color[2];
                }
            }
            image_pub.publish(msg);
        }
    }
};


struct Node {
    ros::NodeHandle nh;
    
    typedef actionlib::SimpleActionServer<object_finder::FindAction> actionserverType;
    actionserverType actionserver;
    
    boost::optional<GoalExecutor> goal_executor;
    
    Node() :
        actionserver(nh, "find", false),
        goal_executor(boost::none) {
        
        actionserver.registerGoalCallback(
            boost::bind(&Node::goalCallback, this));
        actionserver.registerPreemptCallback(
            boost::bind(&Node::preemptCallback, this));
        actionserver.start();
    }
    
    void goalCallback() {
        boost::shared_ptr<const object_finder::FindGoal> new_goal =
            actionserver.acceptNewGoal();
        goal_executor = boost::in_place(*new_goal, boost::bind(boost::mem_fn(
            (void (actionserverType::*)(const actionserverType::Feedback &))
            &actionserverType::publishFeedback), // this was fun
        &actionserver, _1));
    }
    
    void preemptCallback() {
        goal_executor = boost::none;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder");
    
    Node n;
    
    ros::spin();
    
    return 0;
}
