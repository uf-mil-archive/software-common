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

struct PoseGuess {
    TargetDesc goal;
    boost::shared_ptr<const Obj> obj;
    Vector3d pos;
    Quaterniond q;
    RenderBuffer::RegionType bg_region;
    RenderBuffer::RegionType fg_region;
    
    Vector3d last_color;
    
    PoseGuess() { }
    PoseGuess(const TargetDesc &goal,
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
    }
    PoseGuess(const TargetDesc &goal,
             boost::shared_ptr<const Obj> obj,
             Vector3d pos, Quaterniond q) :
        goal(goal), obj(obj),
        pos(pos), q(q) {
    }
    PoseGuess realpredict(double amount) const {
        PoseGuess p(*this);
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
    PoseGuess predict(double amount) const {
        PoseGuess p(*this);
        if(uniform() < .5) {
            return realpredict(.01);
        } else {
            return realpredict(.1);
        }
        return p;
    }
    void draw1(RenderBuffer &renderbuffer, vector<int>* dbg_image=NULL) {
        bg_region = renderbuffer.new_region();
        if(!obj) {
            sphere_draw(renderbuffer, bg_region, pos, 2*goal.sphere_radius,
                dbg_image);
            return;
        }
        BOOST_FOREACH(const Component &component, obj->components) {
            if(component.name.find("background_") == 0) {
                component.draw(renderbuffer, bg_region, pos, q, dbg_image);
            }
        }
    }
    void draw2(RenderBuffer &renderbuffer, vector<int>* dbg_image=NULL) {
        fg_region = renderbuffer.new_region();
        if(!obj) {
            sphere_draw(renderbuffer, fg_region, pos, goal.sphere_radius,
                dbg_image);
            return;
        }
        BOOST_FOREACH(const Component &component, obj->components) {
            if(component.name.find("solid_") == 0) {
                component.draw(renderbuffer, fg_region, pos, q, dbg_image);
            }
        }
    }
    double P(const TaggedImage &img, const vector<ResultWithArea> &results, vector<int>* dbg_image=NULL) {
        Result inner_result = results[fg_region];
        Result outer_result = results[bg_region];
        Result far_result = img.total_result - inner_result; //- outer_result;
        
        if(inner_result.count < 100 || outer_result.count < 100 || far_result.count < 100) {
            return 0.001;
            if(dbg_image) {
                cout << endl;
                cout << endl;
                cout << "NOT VISIBLE" << endl;
            }
            // not visible
        }
        
        last_color = inner_result.avg_color();
        
        Vector3d inner_color = inner_result.avg_color().normalized();
        Vector3d outer_color = outer_result.avg_color().normalized();
        Vector3d far_color = far_result.avg_color().normalized();
        //double P2 = (inner_color - outer_color).norm();
        //double P = P2* pow(far_color.dot(outer_color), 5);
        double P = 1;
        {
            if(results[fg_region].count < 100) {
                if(dbg_image) {
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
                if(dbg_image) {
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

struct Particle {
    double weight;
    vector<PoseGuess> poseguesses;
    vector<double> last_Ps;
    double last_P;
    Particle() { }
    Particle(const object_finder::FindGoal &goal,
             const vector<boost::shared_ptr<Obj> > &objs,
             const TaggedImage &image) {
        
        BOOST_FOREACH(const TargetDesc &targetdesc, goal.targetdescs) {
            poseguesses.push_back(PoseGuess(targetdesc,
                objs[&targetdesc - goal.targetdescs.data()], image));
        }
    }
    Particle predict(double dt, const TaggedImage &image) const {
        Particle p(*this);
        int index = poseguesses.size() * uniform();
        if(uniform() < .5) { // diffuse
            PoseGuess &poseguess = p.poseguesses[index];
            poseguess = poseguess.predict(dt);
        } else { // resample
            PoseGuess &poseguess = p.poseguesses[index];
            poseguess = PoseGuess(poseguess.goal, poseguess.obj, image);
        }
        BOOST_FOREACH(PoseGuess &poseguess, p.poseguesses) {
            bool bad = false;
            BOOST_FOREACH(PoseGuess &poseguess2, p.poseguesses) {
                if(&poseguess2 == &poseguess) break;
                if((poseguess2.pos - poseguess.pos).norm() < .1) {
                    bad = true;
                    break;
                }
            }
            if(bad) {
                poseguess = PoseGuess(poseguess.goal, poseguess.obj, image);
            }
        }
        return p;
    }
    double P(const TaggedImage &img, RenderBuffer &rb, vector<int>* dbg_image=NULL) {
        rb.reset(img);
        BOOST_FOREACH(PoseGuess &poseguess, poseguesses)
            poseguess.draw1(rb, dbg_image);
        BOOST_FOREACH(PoseGuess &poseguess, poseguesses)
            poseguess.draw2(rb, dbg_image);
        vector<ResultWithArea> results = rb.get_results();
        double P = 1;
        last_Ps.clear();
        BOOST_FOREACH(PoseGuess &poseguess, poseguesses) {
            double this_P = poseguess.P(img, results, dbg_image);
            P *= this_P;
            last_Ps.push_back(this_P);
        }
        last_P = P;
        return P;
    }
    double dist(const Particle &other) {
        assert(other.poseguesses.size() == poseguesses.size());
        double max_dist = 0;
        for(unsigned int i = 0; i < poseguesses.size(); i++) {
            max_dist = max(max_dist,
                (poseguesses[i].pos - other.poseguesses[i].pos).norm());
        }
        return max_dist;
    }
};

struct GoalExecutor {
    static const int N = 1000;
    
    ros::NodeHandle nh;
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
    
    std::vector<Particle> particles;
    ros::Time current_stamp;
    
    GoalExecutor(const object_finder::FindGoal &goal,
                 boost::function1<void, const object_finder::FindFeedback&> 
                    feedback_callback) :
        private_nh("~"),
        goal(goal),
        image_sub(nh, "camera/image_rect_color", 1),
        info_sub(nh, "camera/camera_info", 1),
        info_tf_filter(info_sub, listener, "", 10),
        sync(image_sub, info_tf_filter, 10),
        feedback_callback(feedback_callback) {
        
        BOOST_FOREACH(const TargetDesc &targetdesc, goal.targetdescs) {
            current_objs.push_back(targetdesc.type == TargetDesc::TYPE_OBJECT
                ? boost::make_shared<Obj>(targetdesc.object_filename)
                : boost::shared_ptr<Obj>());
        }
        info_tf_filter.setTargetFrame(goal.header.frame_id);
        
        particles_pub = private_nh.advertise<visualization_msgs::Marker>(
            "particles", 1);
        pose_pub = private_nh.advertise<PoseStamped>("pose", 1);
        image_pub = private_nh.advertise<sensor_msgs::Image>(
            "debug_image", 1);
        
        particles.clear();
        sync.registerCallback(boost::bind(&GoalExecutor::callback, this, _1, _2));
    }
    
    void init_particles(ros::Time t, const TaggedImage &img) {
        particles.clear();
        
        for(int i = 0; i < N; i++)
            particles.push_back(Particle(goal, current_objs, img));
        BOOST_FOREACH(Particle &particle, particles)
            particle.weight = 1./particles.size();
        
        current_stamp = t;
    }
    
    static bool compare_weight(const Particle &a, const Particle &b) {
        return a.last_P < b.last_P;
    }
    void callback(const ImageConstPtr& image,
                  const CameraInfoConstPtr& cam_info) {
        assert(image->header.stamp == cam_info->header.stamp);
        assert(image->header.frame_id == cam_info->header.frame_id);
        
        double dt = (image->header.stamp - current_stamp).toSec();
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
        
        // residual resampling
        std::vector<Particle> new_particles;
        BOOST_FOREACH(Particle &particle, particles) {
            int count = N * particle.weight + uniform();
            for(int i = 0; i < count; i++) {
                new_particles.push_back(i==0 ?
                    particle :
                    particle.predict(dt, img));
            }
        }
        particles.swap(new_particles);
        BOOST_FOREACH(Particle &particle, particles)
            particle.weight = 1./particles.size();
        
        // update weights
        {
            RenderBuffer rb(img);
            BOOST_FOREACH(Particle &particle, particles)
                particle.weight *= particle.P(img, rb);
        }
        
        
        // normalize weights
        double total_weight = 0;
        BOOST_FOREACH(Particle &particle, particles) total_weight += particle.weight;
        
        BOOST_FOREACH(Particle &particle, particles) particle.weight /= total_weight;
        
        
        // replace particles doing worse than the median with new ones
        // drawn from the prior distribution
        sort(particles.begin(), particles.end(), compare_weight);
        for(unsigned int i = 0; i < particles.size()/2; i++) {
            Particle &particle = particles[i];
            particle = Particle(goal, current_objs, img);
            particle.weight = 1./particles.size();
        }
        
        
        // find mode
        Particle max_p; max_p.weight = -1;
        BOOST_FOREACH(Particle &particle, particles)
            if(particle.weight > max_p.weight) max_p = particle;
        assert(max_p.weight >= 0);
        
        
        { // send marker message for particle visualization
            visualization_msgs::Marker msg;
            msg.header.frame_id = goal.header.frame_id;
            msg.header.stamp = image->header.stamp;
            msg.type = visualization_msgs::Marker::POINTS;
            msg.scale = make_xyz<Vector3>(.1, .1, 1);
            msg.color.a = 1;
            BOOST_FOREACH(Particle &particle, particles) {
                BOOST_FOREACH(const PoseGuess &poseguess,
                        particle.poseguesses) {
                    msg.points.push_back(vec2xyz<Point>(poseguess.pos));
                    msg.colors.push_back(make_rgba<ColorRGBA>(
                        particle.weight/max_p.weight, 0, 1-particle.weight/max_p.weight, 1));
                }
            }
            particles_pub.publish(msg);
        }
        
        { // send pose message for result visualization
            PoseStamped msg;
            msg.header.frame_id = goal.header.frame_id;
            msg.header.stamp = image->header.stamp;
            msg.pose.position = vec2xyz<Point>(max_p.poseguesses[0].pos);
            msg.pose.orientation = quat2xyzw<geometry_msgs::Quaternion>(
                max_p.poseguesses[0].q);
            pose_pub.publish(msg);
        }
        
        { // send action feedback
            object_finder::FindFeedback feedback;
            BOOST_FOREACH(const PoseGuess &poseguess,
                    max_p.poseguesses) {
                TargetRes targetres;
                targetres.pose.position = vec2xyz<Point>(poseguess.pos);
                targetres.pose.orientation =
                     quat2xyzw<geometry_msgs::Quaternion>(poseguess.q);
                targetres.color = make_rgba<ColorRGBA>(poseguess.last_color[0],
                    poseguess.last_color[1], poseguess.last_color[2], 1);
                if(poseguess.obj) {
                    BOOST_FOREACH(const Marker &marker, poseguess.obj->markers) {
                        targetres.markers.push_back(object_finder::MarkerPoint());
                        targetres.markers[targetres.markers.size()-1].name =
                            marker.name;
                        targetres.markers[targetres.markers.size()-1].position =
                            vec2xyz<Point>(marker.position);
                    }
                }
                feedback.targetreses.push_back(targetres);
            }
            RenderBuffer rb(img);
            feedback.P = max_p.P(img, rb);
            feedback.P_within_10cm = 0;
            BOOST_FOREACH(Particle &particle, particles)
                if(particle.dist(max_p) <= .2)
                    feedback.P_within_10cm += particle.weight;
            feedback_callback(feedback);
        }
        
        if(image_pub.getNumSubscribers()) { // send debug image
            vector<int> dbg_image(image->width*image->height, 0);
            RenderBuffer rb(img);
            max_p.P(img, rb, &dbg_image);
            
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
                        100*dbg_image[image->width*y + x];
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
