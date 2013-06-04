#include <string>
#include <vector>
#include <map>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/range/adaptor/map.hpp>

#include <ros/ros.h>

#include "kill_handling/Kill.h"

namespace kill_handling {

struct KillListener {
    void _killmsg_callback(const KillConstPtr &msg) {
        _kill_cache[msg->id] = make_pair(ros::Time::now(), *msg);
        
        _check_killed();
    }
    
    void _check_killed() {
        bool killed = get_killed();
        if(killed && !_previously_killed)
            _killed_callback();
        else if(!killed && _previously_killed)
            _unkilled_callback();
    }
    
    void _timer_callback(const ros::TimerEvent&) {
        _check_killed();
    }
    
    ros::NodeHandle nh;
    boost::function<void()> _killed_callback;
    boost::function<void()> _unkilled_callback;
    typedef std::pair<ros::Time, Kill> ValueType;
    std::map<std::string, ValueType> _kill_cache;
    ros::Subscriber _sub;
    bool _previously_killed;
    ros::Timer _check_killed_timer;
    
    KillListener(boost::function<void()> killed_callback=boost::function<void()>(), boost::function<void()> unkilled_callback=boost::function<void()>()) {
        _killed_callback = killed_callback;
        _unkilled_callback = unkilled_callback;
        
        _sub = nh.subscribe<Kill>("/kill", 1, boost::bind(&KillListener::_killmsg_callback, this, _1));
        _previously_killed = false;
        _check_killed_timer = nh.createTimer(ros::Duration(.1), boost::bind(&KillListener::_timer_callback, this, _1));
    }
    
    std::vector<std::string> get_kills() {
        ros::Time now = ros::Time::now();
        std::vector<std::string> res;
        BOOST_FOREACH(const ValueType &timekill, _kill_cache | boost::adaptors::map_values) {
            if(timekill.first + timekill.second.lifetime >= now and timekill.second.active) {
                res.push_back(timekill.second.description);
            }
        }
        return res;
    }
    bool get_killed() {
        return this->get_kills().size() != 0;
    }
};

}
