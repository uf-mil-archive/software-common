/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Kevin Hallenbeck
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Kevin Hallenbeck nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ueye/FramerateNode.h>

namespace ueye {

FramerateNode::FramerateNode(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
	// Grab the topic name from the ROS parameter
	std::string topic = std::string("/image_raw");
	priv_nh.getParam("topic", topic);

	// Set up Subscribers
	sub_ = node.subscribe(topic, 2, &FramerateNode::imageRecv, this, ros::TransportHints().tcpNoDelay(true));
}

FramerateNode::~FramerateNode() {}

void FramerateNode::imageRecv(const sensor_msgs::Image::ConstPtr& rosImg)
{
	static double rate = 0.0;
	static long int oldTimeStamp = 0;
	long int newTimeStamp = ros::Time::now().toNSec();
	if(oldTimeStamp != 0){
		double temp_rate = 1000000000.0 / ((double)(newTimeStamp - oldTimeStamp));
		if(rate == 0){
			rate = temp_rate;
		}else{
			rate += (temp_rate - rate) * 0.2;
		}
	}
	oldTimeStamp = newTimeStamp;

	// Convert the ROS Image to an OpenCV Mat
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(rosImg, sensor_msgs::image_encodings::RGB8);

	ROS_INFO("%d %dx%d at %0.2fHz", rosImg->header.seq, cv_ptr->image.cols, cv_ptr->image.rows, rate);
}

} // namespace ueye
