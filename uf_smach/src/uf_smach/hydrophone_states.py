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

class BaseHydrophoneState(smach.State):
    def __init__(self, shared, freq, freq_range=1e3):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])

        self._shared = shared
        self._freq = freq
        self._freq_range = freq_range
        self._ping = None
        self._cond = threading.Condition()

    def execute(self, userdata):
        sub = rospy.Subscriber('/hydrophones/processed', ProcessedPing, self._callback)
        no_ping_ctr = 0

        while not self.preempt_requested():
            with self._cond:
                self._cond.wait(.5)
                if self._ping is None:
                    no_ping_ctr += 1
                    if no_ping_ctr == 5:
                        self._shared['moveto'].send_goal(
                            PoseEditor.from_PoseTwistStamped_topic('/trajectory'))
                    if no_ping_ctr >= 10:
                        return 'failed'
                    continue

                ping = self._ping
                self._ping = None
            no_ping_ctr = 0
            
            goal = self._compute_goal(ping)
            if isinstance(goal, str):
                self._shared['moveto'].send_goal(
                    PoseEditor.from_PoseTwistStamped_topic('/trajectory'))
                return goal
            else:
                self._shared['moveto'].send_goal(goal)
        return 'preempted'

    def _callback(self, processed_ping):
        if abs(processed_ping.freq - self._freq) < self._freq_range and processed_ping.valid:
            with self._cond:
                self._ping = processed_ping
                self._cond.notify()

class HydrophoneTravelState(BaseHydrophoneState):
    def __init__(self, *args, **kwargs):
        BaseHydrophoneState.__init__(self, *args, **kwargs)
        self._stall_ctr = 0
        self._good_ctr = 0

    def execute(self, userdata):
        self._stall_ctr = 0
        self._good_ctr = 0
        return BaseHydrophoneState.execute(self, userdata)
        
    def _compute_goal(self, ping):
        current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        new = current.yaw_left(ping.heading)

        if abs(ping.heading) < 15/180*math.pi:
            self._stall_ctr = max(self._stall_ctr - 1, 0)
            if ping.declination < 30/180*math.pi:
                speed = .7
                self._good_ctr = 0
            elif ping.declination < 40/180*math.pi:
                speed = .3
                self._good_ctr = 0
            else:
                speed = .1
                if ping.declination > 55/180*math.pi:
                    self._good_ctr += 1
                    if self._good_ctr >= 3:
                        return 'succeeded'
        else:
            self._stall_ctr += 1
            if self._stall_ctr > 10:
                return 'failed'
            speed = 0

        if self._stall_ctr > 10:
            return 'failed'
        print 'heading', ping.heading/math.pi*180, 'declination', ping.declination/math.pi*180, 'speed', speed
        return new.as_MoveToGoal(linear=[speed, 0, 0])

class HydrophoneApproachState(BaseHydrophoneState):
    def __init__(self, *args, **kwargs):
        BaseHydrophoneState.__init__(self, *args, **kwargs)
        self._last_declination = 0
    
    def _compute_goal(self, ping):
        current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        if ping.declination < self._last_declination and ping.declination > math.radians(85):
            return 'succeeded'
        self._last_declination = ping.declination
        
        speed = .1 if ping.declination < math.radians(85) else .05
        vel = speed*numpy.array([math.cos(ping.heading), math.sin(ping.heading), 0])

        print 'heading', ping.heading/math.pi*180, 'declination', ping.declination/math.pi*180, 'vel', vel
        return current.as_MoveToGoal(linear=vel.tolist())
