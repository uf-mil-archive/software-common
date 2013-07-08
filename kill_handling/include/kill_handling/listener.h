#ifndef KILL_HANDLING_LISTENER_H
#define KILL_HANDLING_LISTENER_H

#include <string>
#include <vector>
#include <map>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>

#include "kill_handling/KillsStamped.h"
#include "kill_handling/Kill.h"

namespace kill_handling {

struct KillListener {
    void _killmsg_callback(const KillsStampedConstPtr &msg) {
        if (!_kills) {
            _kills = std::map<std::string, Kill>();
        } else {
            _kills->clear();
        }

        BOOST_FOREACH(const Kill &kill, msg->kills) {
            (*_kills)[kill.id] = kill;
        }
        _check_killed();
    }
    
    void _check_killed() {
        bool killed = get_killed();
        if(killed && !_previously_killed) {
            if (_killed_callback)
                _killed_callback();
        } else if(!killed && _previously_killed) {
            if (_unkilled_callback)
                _unkilled_callback();
        }
        _previously_killed = killed;
    }
        
    ros::NodeHandle nh;
    boost::function<void()> _killed_callback;
    boost::function<void()> _unkilled_callback;
    boost::optional<std::map<std::string, Kill> > _kills;
    ros::Subscriber _sub;
    bool _previously_killed;
    ros::Timer _check_killed_timer;
    
    KillListener(boost::function<void()> killed_callback=boost::function<void()>(),
                 boost::function<void()> unkilled_callback=boost::function<void()>()) {
        _killed_callback = killed_callback;
        _unkilled_callback = unkilled_callback;
        
        _sub = nh.subscribe<KillsStamped>("/kill", 1,
                                          boost::bind(&KillListener::_killmsg_callback, this, _1));
        _previously_killed = false;
    }
    
    std::vector<std::string> get_kills() {
        std::vector<std::string> res;
        if (_kills) {
            BOOST_FOREACH(const Kill &kill, *_kills | boost::adaptors::map_values) {
                if(kill.active) {
                    res.push_back(kill.description);
                }
            }
        }
        return res;
    }
    bool get_killed() {
        if (_kills) {
            BOOST_FOREACH(const Kill &kill, *_kills | boost::adaptors::map_values) {
                if (kill.active) {
                    return true;
                }
            }
            return false;
        } else {
            return true;
        }                
    }
};

}

#endif
