from __future__ import division

import threading
import numpy
import math

import rospy
import smach
import smach_ros
import actionlib

import tf
from tf import transformations

from uf_common.orientation_helpers import PoseEditor, xyz_array, xyzw_array
from uf_common.msg import MoveToAction, MoveToGoal
from hydrophones.msg import ProcessedPing

class HydrophoneTravelState(smach.State):
    def __init__(self, shared, freq):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])

        self._shared = shared
        self._freq = freq
        self._ping = None
                                     
    def execute(self, userdata):
        sub = rospy.Subscriber('/hydrophones/processed', ProcessedPing, self._callback)
        no_ping_ctr = 0
        
        while not self.preempt_requested():
            rospy.sleep(1)
            if self._ping is None:
                no_ping_ctr += 1
                if no_ping_ctr == 10:
                    self._shared['moveto'].send_goal(
                        PoseEditor.from_PoseTwistStamped_topic('/trajectory'))
                if no_ping_ctr >= 20:
                    return 'failed'
                continue

            ping = self._ping
            self._ping = None
            no_ping_ctr = 0
            
            if ping.declination > 60/180*math.pi:
                self._shared['moveto'].send_goal(
                    PoseEditor.from_PoseTwistStamped_topic('/trajectory'))
                return 'succeeded'
            
            current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
            new = current.yaw_left(ping.heading)

            if abs(ping.heading) < 10/180*math.pi:
                if ping.declination < 20/180*math.pi:
                    speed = .8
                else:
                    speed = .2
            else:
                speed = 0

            print 'heading', ping.heading/math.pi*180, 'declination', ping.declination/math.pi*180, 'speed', speed
            self._shared['moveto'].send_goal(new.as_MoveToGoal(linear=[speed, 0, 0]))

        if self.preempt_requested():
            return 'preempted'
        return 'succeeded'

    def _callback(self, processed_ping):
        if abs(processed_ping.freq - self._freq) < 1e3:
            self._ping = processed_ping
