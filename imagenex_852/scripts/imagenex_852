#!/usr/bin/env python

from __future__ import division

import math
import sys

import roslib
roslib.load_manifest('imagenex_852')
import rospy
from sensor_msgs.msg import PointCloud2

from imagenex_852 import driver, pointclouds

class Node(object):
    def __init__(self):
        port_filename = rospy.get_param('~port_filename', '/dev/ttyUSB0')
        frame_id = rospy.get_param('~frame_id', '/sonar')
        
        d = driver.Device(port_filename)
        
        pub = rospy.Publisher('imagenex_852', PointCloud2)
        while not rospy.is_shutdown():
            rospy.sleep(.03)
            
            d.send_switch_data_command(train_angle_degrees=0, sector_width_degrees=360, step_size_degrees=3)
            res = d.read_sonar_return_data()
            #res = dict(range_meters=50, echo_data=[52]*252, head_position_degrees=45.2)
            
            angle_rad = math.radians(res['head_position_degrees'])
            scale = res['range_meters']/len(res['echo_data'])
            
            import numpy as np
            array = np.zeros(len(res['echo_data']), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])
            for i, x in enumerate(res['echo_data']):
                array[i]['x'] = scale * i * math.cos(angle_rad)
                array[i]['y'] = scale * i * math.sin(angle_rad)
                array[i]['z'] = 0
                array[i]['intensity'] = x/255
            
            msg = pointclouds.array_to_pointcloud2(array, stamp=rospy.Time.now(), frame_id=frame_id)
            pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imagenex_852')
    try:
        n = Node()
    except rospy.ROSInterruptException:
        pass
