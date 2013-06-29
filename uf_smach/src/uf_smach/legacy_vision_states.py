import json
import math
import threading

import numpy

import rospy
import smach

from uf_common.orientation_helpers import PoseEditor
from legacy_vision.msg import FindAction, FindGoal

class WaitForObjectsState(smach.State):
    def __init__(self, shared, action, object_name=None, timeout=60):
        smach.State.__init__(self, outcomes=['succeeded', 'timeout', 'preempted'])

        self._shared = shared
        self._object_name = object_name
        self._timeout = rospy.Duration(timeout)
        self._cond = threading.Condition()
        self._found = False
        self._action = action

    def execute(self, userdata):
        if self._object_name is not None:
            goal = FindGoal()
            goal.object_names = [self._object_name]
            self._shared[self._action].send_goal(goal, feedback_cb=self._feedback_cb)
        else:
            self._shared[self._action].set_callbacks(feedback_cb=self._feedback_cb)

        end_time = rospy.Time.now() + self._timeout
        with self._cond:
            while not self._found and rospy.Time.now() < end_time and not self.preempt_requested():
                self._cond.wait(1)

        self._shared.clear_callbacks()
        
        if self.preempt_requested():
            return 'preempted'
        return 'succeeded' if self._found else 'timeout'

    def _feedback_cb(self, feedback):
        results = map(json.loads, feedback.targetreses[0].object_results)
        if results:
            with self._cond:
                self._found = True
                self._cond.notify_all()

class BaseManeuverObjectState(smach.State):
    def __init__(self, shared, action, selector=lambda targetreses, traj_start: targetreses[0]):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])

        self._shared = shared
        self._action = action
        self._selector = selector
        self._cond = threading.Condition()
        self._done = False
        self._failed = False
        self._fail_ctr = 0

    def execute(self, userdata):
        self._traj_start = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        self._shared[self._action].set_callbacks(feedback_cb=self._feedback_cb)

        with self._cond:
            while not self._done and not self._failed:
                self._cond.wait()
            self._shared.clear_callbacks()

        return 'succeeded' if self._done else 'failed'

    def _feedback_cb(self, feedback):
        with self._cond:
            good_results = map(json.loads, feedback.targetreses[0].object_results)
            #print good_results
            if len(good_results) == 0:
                self._fail_ctr += 1
                if self._fail_ctr > 10:
                    self._failed = True;
                    self._cond.notify_all()
                return
            else:
                self._fail_ctr = 0

            result = self._selector(good_results, self._traj_start)
            twist = self._get_vel(result)
            if twist is None:
                self._done = True
                self._cond.notify_all()
                return
            linear, angular = twist
            current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
            self._shared['moveto'].send_goal(current.as_MoveToGoal(linear=linear, angular=angular))

class AlignObjectState(BaseManeuverObjectState):
    def _get_vel(self, result):
        vec = numpy.array(map(float, result['center']))
        vec /= numpy.linalg.norm(vec)
        angle = float(result['angle'])
        angle_error = angle - math.radians(90)
        angle_error = (angle_error + math.radians(90)) % math.radians(180) - math.radians(90)
        
        linear = [-1*vec[1], -1*vec[0], 0]
        angular = [0, 0, -.3*angle_error]
        
        if vec.dot([0, 0, 1]) < math.cos(math.radians(5)) and abs(angle_error) < math.radians(5):
            return None
        
        #print vec, linear, angular
        print angle, angle_error
        #if numpy.linalg.norm(linear) < .01:
        #    return None
        
        return linear, angular
