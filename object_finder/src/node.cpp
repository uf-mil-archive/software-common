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
#include "object_finder/TestPose.h"
#include "sphere_finding.h"
#include "obj_finding.h"
#include "image.h"
#include "fft.h"

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
Vector3d normalize_color(Vector3d color) {
  return color/color.sum();
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
    TargetDesc goal;
    Vector3d pos;
    Quaterniond q;
    
    boost::optional<Vector3d> last_color;
    double last_corr;
    
    Particle(TargetDesc const &goal,
             Vector3d pos,
             Quaterniond q,
             boost::optional<Vector3d> last_color=Vector3d(0,0,0),
             double last_corr=nan("")) :
      goal(goal), pos(pos), q(q),
      last_color(last_color), last_corr(last_corr) {
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
                    uniform()*image.width,
                    uniform()*image.height),
                uniform()*(goal.max_dist-goal.min_dist)+goal.min_dist);
        }
        
        Quaterniond q = quat_from_rotvec(dx.tail(3)) *
            xyzw2quat(goal.prior_distribution.pose.orientation);
        if(goal.check_180z_flip && uniform() >= .5)
            q = q * Quaterniond(0, 0, 1, 0);
        
        return Particle(goal, pos, q);
    }
    Particle predict() const {
      double amount = 1; //uniform() < .5 ? 1 : 0.2;
      return Particle(
        goal,
        pos + amount * gaussvec(),
        (!goal.disallow_yawing ? quat_from_rotvec(Vector3d(0, 0, amount * gauss())) : Quaterniond::Identity()) *
        q *
        (goal.allow_pitching ? quat_from_rotvec(Vector3d(0, amount * gauss(), 0)) : Quaterniond::Identity()) *
        (goal.allow_rolling ? quat_from_rotvec(Vector3d(amount * gauss(), 0, 0)) : Quaterniond::Identity()));
    }
    double P(const TaggedImage &img, RenderBuffer &rb, RenderBuffer const &orig_rb, bool print_debug_info=false, std::vector<Particle> * res=NULL) const {
        if(goal.type == TargetDesc::TYPE_SPHERE) {
            RenderBuffer::RegionType fg_region = rb.new_region();
            sphere_draw(rb, fg_region, pos, goal.sphere_radius);
            RenderBuffer::RegionType bg_region = rb.new_region();
            sphere_draw(rb, bg_region, pos, 2*goal.sphere_radius);
            
            std::vector<int> dbg_image(img.width*img.height, 0);
            std::vector<Result> results = rb.draw_debug_regions(dbg_image);
            
            double calculated_corr = 1;
            for(int i = 0; i < 3; i++) {
              double fg_color = Color_to_vec(goal.sphere_color)(i);
              double bg_color = Color_to_vec(goal.sphere_background_color)(i);
              double n = results[fg_region].count + results[bg_region].count;
              double sum_x = results[fg_region].total_color(i) + results[bg_region].total_color(i);
              double sum_x2 = results[fg_region].total_color2(i) + results[bg_region].total_color2(i);
              double sum_y = results[fg_region].count*fg_color + results[bg_region].count*bg_color;
              double sum_y2 = results[fg_region].count*fg_color*fg_color + results[bg_region].count*bg_color*bg_color;
              double sum_xy = results[fg_region].total_color(i)*fg_color + results[bg_region].total_color(i)*bg_color;
              double r = (n*sum_xy - sum_x*sum_y)/sqrt((n*sum_x2 - sum_x*sum_x)*(n*sum_y2 - sum_y*sum_y));
              double r2 = std::max(r, 0.);
              calculated_corr *= r2;
            }
            if(!res) return calculated_corr;
            
            std::vector<Vector4d> colors(10 + std::max(fg_region, bg_region) + 1);
            colors[0] << 0, 0, 0, 0;
            colors[10 + fg_region] << Color_to_vec(goal.sphere_color), 1;
            colors[10 + bg_region] << Color_to_vec(goal.sphere_background_color), 1;
            
            ArrayXXd planes[3];
            for(int i = 0; i < 3; i++) {
              planes[i] = ArrayXXd::Zero(img.height, img.width);
            }
            ArrayXXd weight = ArrayXXd::Zero(img.height, img.width);
            
            unsigned int min_y = img.height, max_y = 0;
            unsigned int min_x = img.height, max_x = 0;
            for(unsigned int y = 0; y < img.height; y++) {
              for(unsigned int x = 0; x < img.width; x++) {
                Vector4d color = colors[dbg_image[y * img.width + x]];
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
                std::cout << std::endl;
                std::cout << std::endl;
                std::cout << "NOT VISIBLE" << std::endl;
              }
              // XXX maybe don't forget about it completely?
              return calculated_corr;
            }
            
            ArrayXXd subweight = weight.block(min_y, min_x, max_y-min_y, max_x-min_x);
            //std::cout << subweight << std::endl << std::endl;
            
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
            
            boost::optional<Particle> maybe_best;
            for(int y = 0; y < img.image[0].rows() - subweight.rows() + 1; y++) {
              for(int x = 0; x < img.image[0].cols() - subweight.cols() + 1; x++) {
                double corr = acc(y, x);
                
                if(!std::isfinite(corr)) {
                  //std::cout << "invalid corr: " << corr << std::endl;
                  continue;
                }
                
                {
                  int offset_y = y - min_y, offset_x = x - min_x;
                  if(offset_y == 0 && offset_x == 0) {
                    std::cout << std::endl;
                    std::cout << "fft corr: " << corr << std::endl;
                    std::cout << "real corr: " << calculated_corr << std::endl;
                    std::cout << std::endl;
                  }
                }
                
                
                if(!maybe_best || corr > maybe_best->last_corr) {
                  int offset_y = y - min_y, offset_x = x - min_x;
                  
                  std::pair<Vector2d, double> old = img.get_point_pixel(pos);
                  Vector3d new_pos = img.get_pixel_point(old.first + Vector2d(offset_x, offset_y), old.second);
                  
                  if(true) { // XXX make configurable
                    rb.reset(img, orig_rb);
                    
                    maybe_best = Particle(goal, new_pos, q, Vector3d(1, 2, 3), -1);
                    maybe_best->last_corr = maybe_best->P(img, rb, rb);
                  }
                }
              }
            }
            
            if(maybe_best) {
              res->push_back(*maybe_best);
            }
            
            {
              rb.reset(img, orig_rb);
              RenderBuffer::RegionType fg_region = rb.new_region();
              sphere_draw(rb, fg_region, pos, goal.sphere_radius);
              RenderBuffer::RegionType bg_region = rb.new_region();
              sphere_draw(rb, bg_region, pos, 2*goal.sphere_radius);
            }
            
            return calculated_corr;
        } else if (goal.type == TargetDesc::TYPE_MESH) {
          const Mesh & mesh = goal.mesh;
          
          std::vector<RenderBuffer::RegionType> regions;
          BOOST_FOREACH(const Component & component, mesh.components) {
            RenderBuffer::RegionType region = rb.new_region();
            obj_finding::draw(component, rb, region, pos, q);
            regions.push_back(region);
          }
          
          std::vector<int> dbg_image(img.width*img.height, 0);
          std::vector<Result> results = rb.draw_debug_regions(dbg_image);
          
          bool skip_channels[3] = {false, false, false};
          for(int i = 0; i < 3; i++) {
            std::vector<double> colors;
            for(unsigned int c = 0; c < mesh.components.size(); c++) {
              double color = (Color_to_vec(mesh.components[c].color))(i);
              colors.push_back(color);
            }
            if(*std::min_element(colors.begin(), colors.end()) ==
               *std::max_element(colors.begin(), colors.end())) {
              skip_channels[i] = true;
            }
          }
          
          double calculated_corr = 1;
          for(int i = 0; i < 3; i++) {
            if(skip_channels[i]) continue;
            double n = 0, sum_x = 0, sum_x2 = 0, sum_y = 0, sum_y2 = 0, sum_xy = 0;
            for(unsigned int c = 0; c < mesh.components.size(); c++) {
              Result const & result = results[regions[c]];
              
              double color = (Color_to_vec(mesh.components[c].color))(i);
              n      += result.count;
              sum_x  += result.total_color(i);
              sum_x2 += result.total_color2(i);
              sum_y  += result.count*color;
              sum_y2 += result.count*color*color;
              sum_xy += result.total_color(i)*color;
            }
            double r = (n*sum_xy - sum_x*sum_y)/sqrt((n*sum_x2 - sum_x*sum_x)*(n*sum_y2 - sum_y*sum_y));
            double r2 = std::max(r, 0.);
            calculated_corr *= r2;
          }
          if(!res) return calculated_corr;
          
          std::vector<Vector4d> colors(10 + *std::max_element(regions.begin(), regions.end()) + 1);
          colors[0] << 0, 0, 0, 0;
          for(unsigned int c = 0; c < mesh.components.size(); c++) {
            RenderBuffer::RegionType region = regions[c];
            
            colors[10 + region] << (Color_to_vec(mesh.components[c].color)), 1;
          }
          
          ArrayXXd planes[3];
          for(int i = 0; i < 3; i++) {
            planes[i] = ArrayXXd::Zero(img.height, img.width);
          }
          ArrayXXd weight = ArrayXXd::Zero(img.height, img.width);
          
          unsigned int min_y = img.height, max_y = 0;
          unsigned int min_x = img.height, max_x = 0;
          for(unsigned int y = 0; y < img.height; y++) {
            for(unsigned int x = 0; x < img.width; x++) {
              Vector4d color = colors[dbg_image[y * img.width + x]];
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
              std::cout << std::endl;
              std::cout << std::endl;
              std::cout << "NOT VISIBLE" << std::endl;
            }
            // XXX maybe don't forget about it completely?
            return calculated_corr;
          }
          
          ArrayXXd subweight = weight.block(min_y, min_x, max_y-min_y, max_x-min_x);
          //std::cout << subweight << std::endl << std::endl;
          
          boost::optional<ArrayXXd> acc;
          for(int i = 0; i < 3; i++) {
            if(skip_channels[i]) continue;
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
            if(!acc) {
              acc = res;
            } else {
              *acc *= res;
            }
          }
          
          boost::optional<Particle> maybe_best;
          for(int y = 0; y < img.image[0].rows() - subweight.rows() + 1; y++) {
            for(int x = 0; x < img.image[0].cols() - subweight.cols() + 1; x++) {
              double corr = (*acc)(y, x);
              
              if(!std::isfinite(corr)) {
                //std::cout << "invalid corr: " << corr << std::endl;
                continue;
              }
              
              {
                int offset_y = y - min_y, offset_x = x - min_x;
                if(offset_y == 0 && offset_x == 0) {
                  std::cout << std::endl;
                  std::cout << "fft corr: " << corr << std::endl;
                  std::cout << "real corr: " << calculated_corr << std::endl;
                  std::cout << std::endl;
                }
              }
              
              
              if(!maybe_best || corr > maybe_best->last_corr) {
                int offset_y = y - min_y, offset_x = x - min_x;
                
                std::pair<Vector2d, double> old = img.get_point_pixel(pos);
                Vector3d new_pos = img.get_pixel_point(old.first + Vector2d(offset_x, offset_y), old.second);
                
                if(true) { // XXX make configurable
                  rb.reset(img, orig_rb);
                  
                  Particle x(goal, new_pos, q, Vector3d(1, 2, 3), -1);
                  x.last_corr = x.P(img, rb, rb);
                  if(std::isfinite(x.last_corr)) {
                    maybe_best = x;
                  }
                }
              }
            }
          }
          
          if(maybe_best) {
            res->push_back(*maybe_best);
          }
          
          {
            rb.reset(img, orig_rb);
            BOOST_FOREACH(const Component & component, mesh.components) {
              RenderBuffer::RegionType region = rb.new_region();
              obj_finding::draw(component, rb, region, pos, q);
            }
          }
          
          return calculated_corr;
        } else {
          assert(false);
        }
    }
    void update(std::vector<Particle> &res, const TaggedImage &img, RenderBuffer &rb, RenderBuffer const &orig_rb, bool print_debug_info=false) const {
        this->P(img, rb, orig_rb, print_debug_info, &res);
    }
    void accumulate_successors(std::vector<Particle> &res, const TaggedImage &img, double N, const std::vector<Particle> &particles, double total_last_corr) const {
        int count = N * last_corr/total_last_corr + uniform();
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
    boost::optional<Particle> previous_best;
    double total_last_corr;
    RenderBuffer myrb;
    
    ParticleFilter(const TargetDesc &targetdesc, const TaggedImage &img, double N) :
        targetdesc(targetdesc) {
        for(int i = 0; i < N; i++)
            particles.push_back(Particle::random(targetdesc, img));
    }
    
    void update(const TaggedImage &img, const RenderBuffer &rb, double N) {
        {
          // residual resampling
          std::vector<Particle> new_particles;
          if(previous_best) {
            new_particles.push_back(*previous_best);
          }
          if(!particles.empty()) {
            Particle best = get_best();
            new_particles.push_back(best);
            for(unsigned int i = 0; i < (N-new_particles.size())/2; i++) {
              new_particles.push_back(best.predict());
            }
            previous_best = best;
          }
          while(new_particles.size() < N) {
            new_particles.push_back(Particle::random(targetdesc, img));
          }
          
          // update particles
          std::vector<Particle> new_particles2;
          BOOST_FOREACH(Particle &particle, new_particles) {
              myrb.reset(img, rb);
              particle.update(new_particles2, img, myrb, rb);
          }
          
          if(new_particles2.empty()) {
            new_particles2.push_back(Particle::random(targetdesc, img));
          }
          
          particles.swap(new_particles2);
        }
        
        total_last_corr = 0;
        BOOST_FOREACH(Particle &particle, particles) total_last_corr += particle.last_corr;
    }
    
    Particle get_best() const {
      Particle const * res = NULL;
      BOOST_FOREACH(const Particle &particle, particles) {
        if(!res || particle.last_corr > res->last_corr) {
          res = &particle;
        }
      }
      assert(res);
      return *res;
    }
};

struct GoalExecutor {
    static constexpr double min_N = 6;
    
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
    ros::ServiceServer test_pose_srv;
    
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
        feedback_callback(feedback_callback),
        test_pose_srv(private_nh.advertiseService("test_pose", boost::function<bool(TestPose::Request&, TestPose::Response&)>(boost::bind(&GoalExecutor::test_pose_callback, this, _1, _2)))) {
        
        BOOST_FOREACH(const TargetDesc &targetdesc, goal.targetdescs) {
            std::ostringstream tmp; tmp << "extracted_images/" << &targetdesc - goal.targetdescs.data();
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
        N = min_N;
        
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
        img = img.decimateBy2().decimateBy2();
        int corner_cut = 0; camera_nh.getParam("corner_cut", corner_cut);
        for(unsigned int i = 0; i < img.height; i++) {
            int dist_from_edge = std::min(i, img.height-1-i);
            int amt = std::max(0, corner_cut - dist_from_edge);
            img.left[i] = amt;
            img.right[i] = img.width - amt;
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
        
        ros::Time start_time = ros::Time::now();
        
        //double total_smoothed_last_P = 0;
        //BOOST_FOREACH(Particle &particle, particles) total_smoothed_last_P += particle.smoothed_last_P;
        
        std::vector<Particle> prev_max_ps;
        BOOST_FOREACH(const ParticleFilter &particle_filter, particle_filters) {
            prev_max_ps.push_back(particle_filter.get_best());
        }
        
        {
            TaggedImage small_img = img; //.decimateBy2().decimateBy2();
            
            BOOST_FOREACH(ParticleFilter &particle_filter, particle_filters) {
                RenderBuffer rb(small_img);
                BOOST_FOREACH(const Particle &p, prev_max_ps) {
                    if(p.last_corr > particle_filter.get_best().last_corr) {
                        p.P(small_img, rb, rb);
                    }
                }
                particle_filter.update(small_img, rb, N);
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
                        particle.last_corr/max_ps[i].last_corr, 0, 1-particle.last_corr/max_ps[i].last_corr, 1));
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
                targetres.P = particle.last_corr;
                targetres.smoothed_last_P = particle.last_corr;
                targetres.P_within_10cm = 0;
                targetres.P_within_10cm_xy = 0;
                ParticleFilter &particle_filter = particle_filters[&particle-max_ps.data()];
                Vector3d c = img.transform * Vector3d::Zero();
                BOOST_FOREACH(Particle &particle2, particle_filter.particles) {
                    if(particle2.dist(particle) <= .2)
                        targetres.P_within_10cm += particle2.last_corr/particle_filter.total_last_corr;
                    double dist = sqrt((particle.pos-c).norm() * (particle2.pos-c).norm());
                    if(((particle2.pos-c).normalized() - (particle.pos-c).normalized()).norm()*dist <= .2)
                        targetres.P_within_10cm_xy += particle2.last_corr/particle_filter.total_last_corr;
                }
                feedback.targetreses.push_back(targetres);
            }
            feedback_callback(feedback);
        }
        
        ros::Time end_time = ros::Time::now();
        std::cout << "took " << (end_time - start_time).toSec()/1e-3 << " ms. N: " << N;
        if(end_time - start_time > ros::Duration(0.25)) {
            N *= .9;
        } else {
            N /= .9;
        }
        if(N < min_N) N = min_N;
        std::cout << " -> " << N << std::endl;
        
        if(image_pub.getNumSubscribers()) { // send debug image
            RenderBuffer rb(img);
            BOOST_FOREACH(const Particle &max_p, max_ps) {
                max_p.P(img, rb, RenderBuffer(img), true);
            }
            
            std::vector<int> dbg_image(img.width*img.height, 0);
            rb.draw_debug_regions(dbg_image);
            
            Image msg;
            msg.header = image->header;
            msg.height = img.height;
            msg.width = img.width;
            msg.encoding = "rgb8";
            msg.is_bigendian = 0;
            msg.step = img.width*3;
            msg.data.resize(img.width*img.height*3);
            for(unsigned int y = 0; y < img.height; y++) {
                for(int x = img.left[y]; x < img.right[y]; x++) {
                    Vector3d orig_color = img.get_pixel(y, x);
                    msg.data[msg.step*y + 3*x + 0] =
                        255.*dbg_image[img.width*y + x]/(rb.areas.size() + 10);
                    msg.data[msg.step*y + 3*x + 1] = 255*orig_color[1];
                    msg.data[msg.step*y + 3*x + 2] = 255*orig_color[2];
                }
            }
            image_pub.publish(msg);
        }
        if(image_pub2.getNumSubscribers()) { // send debug image
            Image msg;
            msg.header = image->header;
            msg.height = img.height;
            msg.width = img.width;
            msg.encoding = "rgb8";
            msg.is_bigendian = 0;
            msg.step = img.width*3;
            msg.data.resize(img.width*img.height*3);
            for(unsigned int y = 0; y < img.height; y++) {
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
                        if(orig_x >= 0 && orig_x < (int)img.width && orig_y >= 0 && orig_y < (int)img.height) {
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
    
    bool test_pose_callback(TestPose::Request &req, TestPose::Response &res) {
      Particle p(goal.targetdescs[0], xyz2vec(req.pose.position), xyzw2quat(req.pose.orientation));
      
      RenderBuffer rb(img);
      res.correlation = p.P(img, rb, RenderBuffer(img), true);
      
      std::vector<int> dbg_image(img.width*img.height, 0);
      rb.draw_debug_regions(dbg_image);
      
      Image msg;
      //msg.header = img.header;
      msg.height = img.height;
      msg.width = img.width;
      msg.encoding = "rgb8";
      msg.is_bigendian = 0;
      msg.step = img.width*3;
      msg.data.resize(img.width*img.height*3);
      for(unsigned int y = 0; y < img.height; y++) {
          for(int x = img.left[y]; x < img.right[y]; x++) {
              Vector3d orig_color = img.get_pixel(y, x);
              msg.data[msg.step*y + 3*x + 0] =
                  255.*dbg_image[img.width*y + x]/(rb.areas.size() + 10);
              msg.data[msg.step*y + 3*x + 1] = 255*orig_color[1];
              msg.data[msg.step*y + 3*x + 2] = 255*orig_color[2];
          }
      }
      image_pub.publish(msg);
      
      return true;
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
