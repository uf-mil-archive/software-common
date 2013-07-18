#include "ueye2/UEyeNode.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

namespace ueye2 {

class UEyeNodelet : public nodelet::Nodelet {
public:
    boost::scoped_ptr<UEyeNode> node;
    boost::thread node_thread;
    volatile bool stop;
    
    UEyeNodelet() : stop(false) { }
    ~UEyeNodelet() {
        stop = true;
        node_thread.join();
    }

    void onInit() {        
        node.reset(new UEyeNode(getNodeHandle(), getPrivateNodeHandle()));
        node_thread = boost::thread(&UEyeNodelet::run, this);
        
    }

    void run() {
        NODELET_INFO("UEye camera running");
        while (!stop) {
            node->runOnce();
        }
    }
};

}

PLUGINLIB_DECLARE_CLASS(ueye2, UEyeNodelet, ueye2::UEyeNodelet, nodelet::Nodelet);
