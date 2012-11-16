#!/usr/bin/env python

from __future__ import division

import math
import sys

import roslib
roslib.load_manifest('imagenex_852')
import rospy
from geometry_msgs.msg import Vector3

from imagenex_852 import driver
from imagenex_852.msg import Echo

class Node(object):
    def __init__(self):
        port_filename = rospy.get_param('~port_filename', '/dev/ttyUSB0')
        frame_id = rospy.get_param('~frame_id', '/sonar')
        
        d = driver.Device(port_filename)
        
        pub = rospy.Publisher('imagenex_852', Echo)
        while not rospy.is_shutdown():
            rospy.sleep(.03)
            
            d.send_switch_data_command(train_angle_degrees=0, sector_width_degrees=360, step_size_degrees=3)
            res = d.read_sonar_return_data()
            #res = dict(_range=50, echo_data=[52]*252, head_position_degrees=45.2)
            
            msg = Echo()
            msg.header.frame_id = frame_id
            msg.header.stamp = rospy.Time.now()
            angle_rad = math.radians(res['head_position_degrees'])
            scale = res['_range']/len(res['echo_data'])
            msg.step = Vector3(scale * math.cos(angle_rad), scale * math.sin(angle_rad), 0)
            msg.intensity = [x/255 for x in res['echo_data']]
            pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('node')
    try:
        n = Node()
    except rospy.ROSInterruptException:
        pass
