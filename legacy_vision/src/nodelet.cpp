#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include "legacy_vision/FindAction.h"
#include "FinderGenerator.h"

using namespace std;

namespace legacy_vision {

static boost::property_tree::ptree ptree_from_xmlrpc(XmlRpc::XmlRpcValue &x) {
    boost::property_tree::ptree res;
    switch(x.getType()) {
        case XmlRpc::XmlRpcValue::TypeBoolean: res.put_value(static_cast<bool>(x)); break;
        case XmlRpc::XmlRpcValue::TypeInt: res.put_value(static_cast<int>(x)); break;
        case XmlRpc::XmlRpcValue::TypeDouble: res.put_value(static_cast<double>(x)); break;
        case XmlRpc::XmlRpcValue::TypeString: res.put_value(static_cast<string>(x)); break;
        case XmlRpc::XmlRpcValue::TypeArray:
            for(int i = 0; i < x.size(); i++) {
                res.push_back(make_pair("", ptree_from_xmlrpc(x[i])));
            }
            break;
        case XmlRpc::XmlRpcValue::TypeStruct:
            for(XmlRpc::XmlRpcValue::iterator it = x.begin(); it != x.end(); it++) {
                res.push_back(make_pair(it->first, ptree_from_xmlrpc(it->second)));
            }
            break;
        default:
            cout << x.getType() << std::endl;
            assert(false); break;
    }
    return res;
}


class GoalExecutor {
public:
    GoalExecutor(const legacy_vision::FindGoal &goal, ros::NodeHandle *nh_, ros::NodeHandle *private_nh_, boost::function1<void, const legacy_vision::FindFeedback&> feedback_callback) :
        nh(*nh_),
        private_nh(*private_nh_),
        camera_nh(nh, "camera"),
        image_sub(camera_nh, "image_rect_color", 1),
        info_sub(camera_nh, "camera_info", 1),
        sync(image_sub, info_sub, 10),
        feedback_callback(feedback_callback) {
        corner_cut = 0; camera_nh.getParam("corner_cut", corner_cut);
        XmlRpc::XmlRpcValue config; private_nh.getParam("", config);
        finders = FinderGenerator::buildFinders(goal.object_names,
            config.getType() == XmlRpc::XmlRpcValue::TypeInvalid ? boost::property_tree::ptree() :
                ptree_from_xmlrpc(config));
        sync.registerCallback(boost::bind(&GoalExecutor::callback, this, _1, _2));
        
        for(unsigned int i = 0; i < goal.object_names.size(); i++) {
            ostringstream tmp; tmp << i;
            res_pubs.push_back(private_nh.advertise<sensor_msgs::Image>("res/" + tmp.str(), 1));
            dbg_pubs.push_back(private_nh.advertise<sensor_msgs::Image>("dbg/" + tmp.str(), 1));
        }
    }
private:
    typedef pair<string, boost::shared_ptr<IFinder> > FinderPair;
    vector<FinderPair> finders;
    ros::NodeHandle &nh;
    ros::NodeHandle &private_nh;
    ros::NodeHandle camera_nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync;
    boost::function1<void, const legacy_vision::FindFeedback&> feedback_callback;
    int corner_cut;
    vector<ros::Publisher> res_pubs;
    vector<ros::Publisher> dbg_pubs;
    void callback(const sensor_msgs::ImageConstPtr& image,
                  const sensor_msgs::CameraInfoConstPtr& cam_info) {
        assert(image->header.stamp == cam_info->header.stamp);
        assert(image->header.frame_id == cam_info->header.frame_id);
        
        cv_bridge::CvImagePtr cvimage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        for(int dx = -1; dx <= 1; dx += 2) {
            for(int dy = -1; dy <= 1; dy += 2) {
                for(unsigned int y = 0; y < cam_info->height; y++) {
                    int x = corner_cut - (dy > 0 ? cam_info->height-1 - y : y);
                    if(dx > 0) x = cam_info->width-1 - x;
                    for(int c = 1; c++; ) {
                        int i = y + dy*c, j = x + dx*c;
                        if(i < 0 || j < 0 || i >= cam_info->height || j >= cam_info->width) break;
                        cvimage->image.at<cv::Vec3b>(i, j) = cvimage->image.at<cv::Vec3b>(y, x);
                    }
                    x += dx;
                    for(int c = 1; c++; ) {
                        int i = y + dy*c, j = x + dx*c;
                        if(i < 0 || j < 0 || i >= cam_info->height || j >= cam_info->width) break;
                        cvimage->image.at<cv::Vec3b>(i, j) = cvimage->image.at<cv::Vec3b>(y, x);
                    }
                }
            }
        }
        subjugator::ImageSource::Image img(cvimage->image, *cam_info);
        
        FindFeedback feedback;
        feedback.header = image->header;
        BOOST_FOREACH(const FinderPair &finder, finders) { int i = &finder - finders.data();
            cout << "Looking for " << finder.first << endl;
            
            vector<boost::property_tree::ptree> fResult;
            boost::optional<cv::Mat> res;
            boost::optional<cv::Mat> dbg;
            try {
                IFinder::FinderResult result = finder.second->find(img);
                fResult = result.results;
                res = result.res;
                dbg = result.dbg;
            } catch(const std::exception &exc) {
                boost::property_tree::ptree error_result;
                error_result.put("objectName", "error");
                error_result.put("what", exc.what());
                fResult.push_back(error_result);
            }
            
            TargetRes targetres;
            boost::property_tree::ptree results;
            BOOST_FOREACH(const boost::property_tree::ptree &pt, fResult) {
                ostringstream s; boost::property_tree::json_parser::write_json(s, pt);
                cout << "Found object: " << s.str() << endl;
                
                targetres.object_results.push_back(s.str());
            }
            
            feedback.targetreses.push_back(targetres);
            
            if(res && res_pubs[i].getNumSubscribers()) {
                res_pubs[i].publish(cv_bridge::CvImage(cvimage->header, cvimage->encoding, *res).toImageMsg());
            }
            
            if(dbg && dbg_pubs[i].getNumSubscribers()) {
                dbg_pubs[i].publish(cv_bridge::CvImage(cvimage->header, "mono8", *dbg).toImageMsg());
            }
        }
        
        feedback_callback(feedback);
    }
};


class NodeImpl {
public:
    NodeImpl(ros::NodeHandle *nh_, ros::NodeHandle *private_nh_) :
        nh(*nh_),
        private_nh(*private_nh_),
        actionserver(nh, "find", false),
        goal_executor(boost::none) {
        
        actionserver.registerGoalCallback(
            boost::bind(&NodeImpl::goalCallback, this));
        actionserver.registerPreemptCallback(
            boost::bind(&NodeImpl::preemptCallback, this));
        actionserver.start();
    }
    
    ros::NodeHandle &nh;
    ros::NodeHandle &private_nh;
    
    typedef actionlib::SimpleActionServer<legacy_vision::FindAction> actionserverType;
    actionserverType actionserver;
    
private:
    boost::optional<GoalExecutor> goal_executor;
    
    void goalCallback() {
        boost::shared_ptr<const legacy_vision::FindGoal> new_goal =
            actionserver.acceptNewGoal();
        
        try {
            goal_executor = boost::in_place(*new_goal, &nh, &private_nh, boost::bind(boost::mem_fn(
                (void (actionserverType::*)(const actionserverType::Feedback &))
                &actionserverType::publishFeedback), // this was fun
            &actionserver, _1));
        } catch(const std::exception &exc) {
            FindResult result;
            result.error = exc.what();
            actionserver.setAborted(result);
            goal_executor = boost::none;
        }
    }
    
    void preemptCallback() {
        goal_executor = boost::none;
        actionserver.setPreempted();
    }
};

class Nodelet : public nodelet::Nodelet {
public:
    Nodelet() { }
    
    virtual void onInit() {
        nodeimpl = boost::in_place(&getNodeHandle(), &getPrivateNodeHandle());
    }

private:
    boost::optional<NodeImpl> nodeimpl;
};
PLUGINLIB_DECLARE_CLASS(legacy_vision, nodelet, legacy_vision::Nodelet, nodelet::Nodelet);

}
