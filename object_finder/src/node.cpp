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
#include "fft.h"

using namespace std;
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

Vector3d Color_to_vec(Color c) {
    return Vector3d(c.r, c.g, c.b);
}

template<int N>
Eigen::Matrix<double, N, N> cholesky(Eigen::Matrix<double, N, N> x) {
  Eigen::LDLT<Eigen::Matrix<double, N, N> > ldlt = x.ldlt();
  Eigen::Array<double, N, 1> d(ldlt.vectorD().array());
  for(int i = 0; i < N; i++) {
    if(d[i] < 0) d[i] = 0;
  }
  Eigen::Matrix<double, N, 1> sqrtd(d.sqrt());
  return ldlt.transpositionsP().transpose() * Eigen::Matrix<double, N, N>(ldlt.matrixL()) * sqrtd.asDiagonal();
}

template<typename T>
T make_rgb(float r, float g, float b) {
  T res;
  res.r = r;
  res.g = g;
  res.b = b;
  return res;
}


struct Particle {
    TargetDesc const goal;
    Vector3d const pos;
    Quaterniond const q;
    
    boost::optional<Vector3d> const last_color;
    std::vector<double> const past_Ps;
    double const past_Ps_sum;
    
    Particle(TargetDesc const &goal,
             Vector3d pos,
             Quaterniond q,
             boost::optional<Vector3d> last_color=boost::none,
             std::vector<double> past_Ps=std::vector<double>()) :
      goal(goal), pos(pos), q(q), last_color(last_color), past_Ps(past_Ps),
      past_Ps_sum(std::accumulate(past_Ps.begin(), past_Ps.end(), 0.)) {
      assert(past_Ps.size() <= 10);
    }
    static Particle random(TargetDesc const &goal,
                           TaggedImage const &image) {
        Matrix6d cov = Map<const Matrix6d>(
            goal.prior_distribution.covariance.data());
        Matrix6d dist_from_iid = cholesky(cov);
        Vector6d iid; iid << gaussvec(), gaussvec();
        Vector6d dx = dist_from_iid * iid;
        
        Vector3d pos = dx.head(3) + xyz2vec(goal.prior_distribution.pose.position);
        if(goal.max_dist) {
            pos = image.get_pixel_point(
                Vector2d(
                    uniform()*image.cam_info.width,
                    uniform()*image.cam_info.height),
                uniform()*(goal.max_dist-goal.min_dist)+goal.min_dist);
        }
        
        Quaterniond q = quat_from_rotvec(dx.tail(3)) *
            xyzw2quat(goal.prior_distribution.pose.orientation);
        if(goal.check_180z_flip && uniform() >= .5)
            q = q * Quaterniond(0, 0, 1, 0);
        
        return Particle(goal, pos, q);
    }
    Particle realpredict(double amount) const {
        return Particle(
          goal,
          pos + amount * gaussvec(),
          (!goal.disallow_yawing ? quat_from_rotvec(Vector3d(0, 0, amount * gauss())) : Quaterniond::Identity()) *
          q *
          (goal.allow_pitching ? quat_from_rotvec(Vector3d(0, amount * gauss(), 0)) : Quaterniond::Identity()) *
          (goal.allow_rolling ? quat_from_rotvec(Vector3d(amount * gauss(), 0, 0)) : Quaterniond::Identity()));
    }
    Particle predict() const {
        if(uniform() < .5) {
            return realpredict(.02);
        } else {
            return realpredict(.06);
        }
    }
    void P(const TaggedImage &img, RenderBuffer &rb, bool print_debug_info=false, std::vector<Particle> * res=NULL) const {
        if(goal.type == TargetDesc::TYPE_SPHERE) {
            RenderBuffer::RegionType fg_region = rb.new_region();
            sphere_draw(rb, fg_region, pos, goal.sphere_radius);
            RenderBuffer::RegionType bg_region = rb.new_region();
            sphere_draw(rb, bg_region, pos, 2*goal.sphere_radius);
            
            std::vector<Vector4d> colors(10 + bg_region + 1);
            colors[0] << 0, 0, 0, 0;
            colors[10 + fg_region] << Color_to_vec(goal.sphere_color), 1;
            colors[10 + bg_region] << Color_to_vec(goal.sphere_background_color), 1;
            
            vector<int> dbg_image(img.cam_info.width*img.cam_info.height, 0);
            vector<Result> x = rb.draw_debug_regions(dbg_image);
            
            ArrayXXd planes[3];
            for(int i = 0; i < 3; i++) {
              planes[i] = ArrayXXd::Zero(img.cam_info.height, img.cam_info.width);
            }
            ArrayXXd weight = ArrayXXd::Zero(img.cam_info.height, img.cam_info.width);
            
            int min_y = img.cam_info.height, max_y = 0;
            int min_x = img.cam_info.height, max_x = 0;
            for(int y = 0; y < img.cam_info.height; y++) {
              for(int x = 0; x < img.cam_info.width; x++) {
                Vector4d color = colors[dbg_image[y * img.cam_info.width + x]];
                if(!color(3)) continue;
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                for(int i = 0; i < 3; i++) {
                  planes[i](y, x) = color(i);
                }
                weight(y, x) = color(3);
              }
            }
            
            if(min_y >= max_y || min_x >= max_x) {
              // not visible
              if(print_debug_info) {
                cout << endl;
                cout << endl;
                cout << "NOT VISIBLE" << endl;
              }
              // XXX maybe don't forget about it completely?
            }
            
            ArrayXXd subweight = weight.block(min_y, min_x, max_y-min_y, max_x-min_x);
            
            ArrayXXd acc;
            for(int i = 0; i < 3; i++) {
              ArrayXXd res;
              fft::calc_pcc(img.image[i],
                            planes[i].block(min_y, min_x, max_y-min_y, max_x-min_x),
                            subweight,
                            0,
                            0,
                            img.image[0].rows() - subweight.rows() + 1,
                            img.image[0].cols() - subweight.cols() + 1,
                            &res);
              res = res.max(0);
              if(i == 0) {
                acc = res;
              } else {
                acc *= res;
              }
            }
            
            for(int y = 0; y < img.image[0].rows() - subweight.rows() + 1; y++) {
              for(int x = 0; x < img.image[0].cols() - subweight.cols() + 1; x++) {
                int offset_y = y - min_y, offset_x = x - min_x;
                
                std::pair<Vector2d, double> old = img.get_point_pixel(pos);
                Vector3d new_pos = img.get_pixel_point(old.first + Vector2d(offset_x, offset_y), old.second);
                res->emplace_back(goal, new_pos, q, Vector3d(1, 2, 3));
                
                // XXX need to set last_color
                // XXX need to set score
                // XXX need to make sure particle isn't definitely eliminated next round ..
              }
            }
            
            return;
        }
        
        assert(false);
        
        /*
        typedef std::pair<const Component *, RenderBuffer::RegionType> Pair;
        static std::vector<Pair> regions; regions.clear();
        BOOST_FOREACH(const Component &component, goal.mesh.components) {
            if(component.name.find("marker ") == 0 || component.name.find("ignore ") == 0) continue;
            RenderBuffer::RegionType region = rb.new_region();
            obj_finding::draw(component, rb, region, pos, q);
            regions.push_back(make_pair(&component, region));
        }
        
        
        vector<ResultWithArea> results = rb.get_results();
        
        double P = 1;
        BOOST_FOREACH(const Pair &p1, regions) {
            const ResultWithArea &p1_result = results[p1.second];
            BOOST_FOREACH(const Pair &p2, regions) {
                if(&p2 >= &p1) continue;
                const ResultWithArea &p2_result = results[p2.second];
                
                istringstream p1_ss(p1.first->name);
                string p1_prefix; p1_ss >> p1_prefix;
                string p1_op; p1_ss >> p1_op;
                Vector3d p1_dcolor; p1_ss >> p1_dcolor[0] >> p1_dcolor[1] >> p1_dcolor[2];
                
                istringstream p2_ss(p2.first->name);
                string p2_prefix; p2_ss >> p2_prefix;
                string p2_op; p2_ss >> p2_op;
                Vector3d p2_dcolor; p2_ss >> p2_dcolor[0] >> p2_dcolor[1] >> p2_dcolor[2];
                
                if(p2_dcolor == p1_dcolor) continue;
                Vector3d dir = (p2_dcolor - p1_dcolor).normalized();
                
                if(p1_result.count < 10 || p2_result.count < 10) {
                    P *= 0;
                    continue;
                }
                
                P *= exp(30*(p2_result.avg_color_assuming_unseen_is(p1_result.avg_color()) - p1_result.avg_color_assuming_unseen_is(p2_result.avg_color())).dot(dir));
            }
        }
        BOOST_FOREACH(const Pair &p1, regions) {
            const ResultWithArea &p1_result = results[p1.second];
            istringstream p1_ss(p1.first->name);
            string p1_prefix; p1_ss >> p1_prefix;
            string p1_op; p1_ss >> p1_op;
            if(p1_op == "colornovar") {
                if(p1_result.count == 0) {
                    P *= 1e-3;
                    continue;
                }
                double var = 0;
                for(int i = 0; i < 3; i++) {
                    var += p1_result.total_color2[i]/p1_result.count - pow(p1_result.total_color[i]/p1_result.count, 2);
                }
                P *= exp(-1e3*var);
                if(print_debug_info) {
                    cout << "var " << var << " " << p1_result.total_color2.transpose() << " / " << p1_result.total_color.transpose() << " count " << p1_result.count << endl;
                }
            }
        }
        if(print_debug_info) {
            cout << "P " << P << endl;
        }
        
        if(!(isfinite(P) && P >= 0)) {
            cout << "bad P: " << P << endl;
            throw std::runtime_error("bad P");
        }
        return P;
        */
    }
    void update(std::vector<Particle> &res, const TaggedImage &img, RenderBuffer &rb, bool print_debug_info=false) const {
        this->P(img, rb, print_debug_info, &res);
    }
    void accumulate_successors(std::vector<Particle> &res, const TaggedImage &img, double N, const std::vector<Particle> &particles, double total_past_Ps_sum) const {
        int count = N * past_Ps_sum/total_past_Ps_sum + uniform();
        for(int i = 0; i < count; i++) {
            res.push_back(i==0 ?
                *this :
                predict());
        }
    }
    double dist(const Particle &other) const {
        return (pos - other.pos).norm();
    }
};

struct ParticleFilter {
    TargetDesc targetdesc;
    std::vector<Particle> particles;
    double total_past_Ps_sum;
    RenderBuffer myrb;
    
    ParticleFilter(const TargetDesc &targetdesc, const TaggedImage &img, double N) :
        targetdesc(targetdesc) {
        for(int i = 0; i < N; i++)
            particles.push_back(Particle::random(targetdesc, img));
    }
    
    void update(const TaggedImage &img, const RenderBuffer &rb, double N) {
        double original_total_past_Ps_sum = 0;
        BOOST_FOREACH(const Particle &particle, particles) original_total_past_Ps_sum += particle.past_Ps_sum;
        
        {
          // residual resampling
          std::vector<Particle> new_particles;
          BOOST_FOREACH(const Particle &particle, particles) {
              particle.accumulate_successors(new_particles, img, N*2/3., particles, original_total_past_Ps_sum);
          }
          for(unsigned int i = 0; i < N/3; i++) {
              new_particles.push_back(Particle::random(targetdesc, img));
          }
          
          // update particles
          std::vector<Particle> new_particles2;
          BOOST_FOREACH(Particle &particle, new_particles) {
              myrb.reset(img, rb);
              particle.update(new_particles2, img, myrb);
          }
          
          particles.swap(new_particles2);
        }
        
        total_past_Ps_sum = 0;
        BOOST_FOREACH(Particle &particle, particles) total_past_Ps_sum += particle.past_Ps_sum;
    }
    
    Particle get_best() const {
      Particle const * res = NULL;
      BOOST_FOREACH(const Particle &particle, particles) {
        if(particle.past_Ps.size() < 10) continue;
        if(!res || particle.past_Ps_sum > res->past_Ps_sum) {
          res = &particle;
        }
      }
      if(res) {
        return *res;
      }
      BOOST_FOREACH(const Particle &particle, particles) {
        if(!res || particle.past_Ps_sum > res->past_Ps_sum) {
          res = &particle;
        }
      }
      return *res;
    }
};

struct GoalExecutor {
    ros::NodeHandle nh;
    ros::NodeHandle camera_nh;
    ros::NodeHandle private_nh;
    FindGoal goal;
    
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<CameraInfo> info_sub;
    tf::MessageFilter<CameraInfo> info_tf_filter;
    TimeSynchronizer<Image, CameraInfo> sync;
    ros::Publisher particles_pub;
    ros::Publisher pose_pub;
    ros::Publisher image_pub;
    ros::Publisher image_pub2;
    std::vector<ros::Publisher> extracted_image_pubs;
    boost::function1<void, const FindFeedback&> feedback_callback;
    
    double N;
    TaggedImage img;
    std::vector<ParticleFilter> particle_filters;
    ros::Time current_stamp;
    
    GoalExecutor(const FindGoal &goal,
                 boost::function1<void, const FindFeedback&> 
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
            ostringstream tmp; tmp << "extracted_images/" << &targetdesc - goal.targetdescs.data();
            extracted_image_pubs.push_back(private_nh.advertise<sensor_msgs::Image>(tmp.str(), 1));
        }
        
        particles_pub = private_nh.advertise<visualization_msgs::Marker>(
            "particles", 1);
        pose_pub = private_nh.advertise<PoseStamped>("pose", 1);
        image_pub = private_nh.advertise<sensor_msgs::Image>(
            "debug_image", 1);
        image_pub2 = private_nh.advertise<sensor_msgs::Image>(
            "debug_image2", 1);
        
        sync.registerCallback(boost::bind(&GoalExecutor::callback, this, _1, _2));
    }
    
    void init(ros::Time t, const TaggedImage &img) {
        N = 10;
        
        particle_filters.clear();
        BOOST_FOREACH(const TargetDesc &targetdesc, goal.targetdescs) {
            particle_filters.push_back(ParticleFilter(targetdesc, img, N));
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
        
        img.reset(*image, *cam_info, eigen_from_tf(transform));
        int corner_cut = 0; camera_nh.getParam("corner_cut", corner_cut);
        for(unsigned int i = 0; i < img.cam_info.height; i++) {
            int dist_from_edge = min(i, img.cam_info.height-1-i);
            int amt = max(0, corner_cut - dist_from_edge);
            img.left[i] = amt;
            img.right[i] = img.cam_info.width - amt;
        }
        
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
        
        std::vector<Particle> prev_max_ps;
        BOOST_FOREACH(const ParticleFilter &particle_filter, particle_filters) {
            prev_max_ps.push_back(particle_filter.get_best());
        }
        
        {
            BOOST_FOREACH(ParticleFilter &particle_filter, particle_filters) {
                RenderBuffer rb(img);
                BOOST_FOREACH(const Particle &p, prev_max_ps) {
                    if(p.past_Ps_sum > particle_filter.get_best().past_Ps_sum) {
                        p.P(img, rb);
                    }
                }
                particle_filter.update(img, rb, N);
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
                    msg.colors.push_back(make_rgba<std_msgs::ColorRGBA>(
                        particle.past_Ps_sum/max_ps[i].past_Ps_sum, 0, 1-particle.past_Ps_sum/max_ps[i].past_Ps_sum, 1));
                }
             i++; }
            particles_pub.publish(msg);
        }
        
        if(max_ps.size()) { // send pose message for result visualization
            PoseStamped msg;
            msg.header.frame_id = goal.header.frame_id;
            msg.header.stamp = image->header.stamp;
            msg.pose.position = vec2xyz<Point>(max_ps[0].pos);
            msg.pose.orientation = quat2xyzw<geometry_msgs::Quaternion>(
                max_ps[0].q);
            pose_pub.publish(msg);
        }
        
        { // send action feedback
            FindFeedback feedback;
            BOOST_FOREACH(const Particle &particle, max_ps) {
                TargetRes targetres;
                targetres.pose.position = vec2xyz<Point>(particle.pos);
                targetres.pose.orientation =
                     quat2xyzw<geometry_msgs::Quaternion>(particle.q);
                targetres.color = make_rgb<Color>((*particle.last_color)[0],
                    (*particle.last_color)[1], (*particle.last_color)[2]);
                targetres.P = particle.past_Ps.size() ? particle.past_Ps[0] : -1.;
                targetres.smoothed_last_P = particle.past_Ps_sum;
                targetres.P_within_10cm = 0;
                targetres.P_within_10cm_xy = 0;
                ParticleFilter &particle_filter = particle_filters[&particle-max_ps.data()];
                Vector3d c = img.transform * Vector3d::Zero();
                BOOST_FOREACH(Particle &particle2, particle_filter.particles) {
                    if(particle2.dist(particle) <= .2)
                        targetres.P_within_10cm += particle2.past_Ps_sum/particle_filter.total_past_Ps_sum;
                    double dist = sqrt((particle.pos-c).norm() * (particle2.pos-c).norm());
                    if(((particle2.pos-c).normalized() - (particle.pos-c).normalized()).norm()*dist <= .2)
                        targetres.P_within_10cm_xy += particle2.past_Ps_sum/particle_filter.total_past_Ps_sum;
                }
                feedback.targetreses.push_back(targetres);
            }
            feedback_callback(feedback);
        }
        
        ros::WallTime end_time = ros::WallTime::now();
        if(end_time - start_time > ros::WallDuration(5)) {
            N *= .9;
        } else {
            N /= .9;
        }
        cout << "N = " << N << endl;
        
        if(image_pub.getNumSubscribers()) { // send debug image
            RenderBuffer rb(img);
            BOOST_FOREACH(const Particle &max_p, max_ps) {
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
                for(int x = img.left[y]; x < img.right[y]; x++) {
                    Vector3d orig_color = img.get_pixel(y, x);
                    msg.data[msg.step*y + 3*x + 0] =
                        255.*dbg_image[image->width*y + x]/(rb.areas.size() + 10);
                    msg.data[msg.step*y + 3*x + 1] = 255*orig_color[1];
                    msg.data[msg.step*y + 3*x + 2] = 255*orig_color[2];
                }
            }
            image_pub.publish(msg);
        }
        if(image_pub2.getNumSubscribers()) { // send debug image
            Image msg;
            msg.header = image->header;
            msg.height = image->height;
            msg.width = image->width;
            msg.encoding = "rgb8";
            msg.is_bigendian = 0;
            msg.step = image->width*3;
            msg.data.resize(image->width*image->height*3);
            for(unsigned int y = 0; y < image->height; y++) {
                for(int x = img.left[y]; x < img.right[y]; x++) {
                    Vector3d orig_color = img.get_pixel(y, x);
                    msg.data[msg.step*y + 3*x + 0] = 255*orig_color[0];
                    msg.data[msg.step*y + 3*x + 1] = 255*orig_color[1];
                    msg.data[msg.step*y + 3*x + 2] = 255*orig_color[2];
                }
            }
            image_pub2.publish(msg);
        }
        BOOST_FOREACH(const Particle &max_p, max_ps) { // send contained images
            ros::Publisher pub = extracted_image_pubs[&max_p - max_ps.data()];
            if(!pub.getNumSubscribers()) continue;
            
            Image msg;
            msg.header.stamp = image->header.stamp;
            msg.height = max_p.goal.extract_y_pixels;
            msg.width = max_p.goal.extract_x_pixels;
            msg.encoding = "rgb8";
            msg.is_bigendian = 0;
            msg.step = msg.width*3;
            msg.data.resize(msg.width*msg.height*3);
            for(unsigned int y = 0; y < msg.height; y++) {
                for(unsigned int x = 0; x < msg.width; x++) {
                    Vector3d pos_body = xyz2vec(max_p.goal.extract_origin) + (x+.5)/(double)msg.width * xyz2vec(max_p.goal.extract_x) + (y+.5)/(double)msg.height * xyz2vec(max_p.goal.extract_y);
                    Vector3d c0_camera = img.transform_inverse * (max_p.pos + max_p.q._transformVector(pos_body));
                    Vector3d c0_homo = img.proj * c0_camera.homogeneous();
                    
                    Vector3d orig_color = Vector3d::Zero();
                    if(c0_homo(2) > 0) { // check if behind camera
                        Vector2d c0 = c0_homo.hnormalized();
                        
                        int orig_x = c0(0) + .5, orig_y = c0(1) + .5;
                        if(orig_x >= 0 && orig_x < (int)img.cam_info.width && orig_y >= 0 && orig_y < (int)img.cam_info.height) {
                            orig_color = img.get_pixel(orig_y, orig_x);
                        }
                    }
                    
                    msg.data[msg.step*y + 3*x + 0] = 255*orig_color[0];
                    msg.data[msg.step*y + 3*x + 1] = 255*orig_color[1];
                    msg.data[msg.step*y + 3*x + 2] = 255*orig_color[2];
                }
            }
            pub.publish(msg);
        }
    }
};


struct Node {
    ros::NodeHandle nh;
    
    typedef actionlib::SimpleActionServer<FindAction> actionserverType;
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
        boost::shared_ptr<const FindGoal> new_goal =
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
