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
        if(_kill_cache.count(msg->id) && msg->header.stamp < _kill_cache[msg->id].header.stamp) {
            return; // this is older than current info
        }
        
        _kill_cache[msg->id] = *msg;
        
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
    std::map<std::string, Kill> _kill_cache;
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
        ros::Time t = ros::Time::now();
        std::vector<std::string> res;
        BOOST_FOREACH(const Kill &kill, _kill_cache | boost::adaptors::map_values) {
            if(kill.header.stamp + kill.lifetime >= t and kill.active) {
                res.push_back(kill.description);
            }
        }
        return res;
    }
    bool get_killed() {
        return this->get_kills().size() != 0;
    }
};

}
