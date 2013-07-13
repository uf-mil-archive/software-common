from __future__ import division

import json
import math
import threading

import numpy

import rospy
import smach
import tf
from tf import transformations

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
    def __init__(self, shared, action, selector=lambda targetreses, traj_start, (tf_p, tf_q): targetreses[0]):
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
            while not (self._done) and not self._failed:
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

            current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
            try:
                print current.frame_id, feedback.header.frame_id, feedback.header.stamp-rospy.Time.now(), rospy.Duration(1)
                self._shared['tf_listener'].waitForTransform(current.frame_id, feedback.header.frame_id, feedback.header.stamp, rospy.Duration(1))
                world_from_result_tf = self._shared['tf_listener'].lookupTransform(current.frame_id, feedback.header.frame_id, feedback.header.stamp)
            except ValueError: #tf.Exception, e:
                import traceback
                traceback.print_exc()
                return
            
            result = self._selector(good_results, self._traj_start, world_from_result_tf)
            
            goal = self._get_goal(result, current, world_from_result_tf)
            if goal is None:
                self._done = True
                self._cond.notify_all()
                return
            self._shared['moveto'].send_goal(goal)

class CenterObjectState(BaseManeuverObjectState):
    def _get_goal(self, result, current, (tf_p, tf_q)):
        vec = numpy.array(map(float, result['center'])); vec /= numpy.linalg.norm(vec)
        vec_world = transformations.quaternion_matrix(tf_q)[:3, :3].dot(vec)
        camera_axis = transformations.quaternion_matrix(tf_q)[:3, :3].dot([0, 0, 1])
        
        if vec_world.dot(camera_axis) > math.cos(math.radians(1)):
            # if it's within a 2 degree cone of camera axis, terminate
            return None
        
        # get rid of component going along camera axis
        vec_world2 = vec_world - camera_axis*camera_axis.dot(vec_world)
        
        vel_world = 2*vec_world2
        if numpy.linalg.norm(vel_world) > .2:
            vel_world = .2 * vel_world/numpy.linalg.norm(vel_world)
        return current.as_MoveToGoal(linear=current._rot.T.dot(vel_world))

class CenterApproachObjectState(BaseManeuverObjectState):
    def __init__(self, *args, **kwargs):
        self._desired_scale = kwargs.pop('desired_scale')
        BaseManeuverObjectState.__init__(self, *args, **kwargs)

    def _get_goal(self, result, current, (tf_p, tf_q)):
        approach_vel = (self._desired_scale - float(result['scale']))/self._desired_scale
        approach_vel = max(min(approach_vel, .2), -.2)
        print float(result['scale']), approach_vel
        
        vec = numpy.array(map(float, result['center'])); vec /= numpy.linalg.norm(vec)
        vec_world = transformations.quaternion_matrix(tf_q)[:3, :3].dot(vec)
        camera_axis = transformations.quaternion_matrix(tf_q)[:3, :3].dot([0, 0, 1])
        
        if vec_world.dot(camera_axis) > math.cos(math.radians(1)) and abs(approach_vel) < .01:
            # if it's within a 2 degree cone of camera axis, terminate
            return None
        
        # get rid of component going along camera axis
        vec_world2 = vec_world - camera_axis*camera_axis.dot(vec_world)
        
        vel_world = 2*vec_world2
        if numpy.linalg.norm(vel_world) > .2:
            vel_world = .2 * vel_world/numpy.linalg.norm(vel_world)
        else:
            vel_world += approach_vel*camera_axis
        return current.as_MoveToGoal(linear=current._rot.T.dot(vel_world))

class AlignObjectState(BaseManeuverObjectState):
    def __init__(self, *args, **kwargs):
        self._body_vec_align = kwargs.pop('body_vec_align', [1, 0, 0])
        BaseManeuverObjectState.__init__(self, *args, **kwargs)
    
    def _get_goal(self, result, current, (tf_p, tf_q)):
        direction = numpy.array(map(float, result['direction']))
        direction_world = transformations.quaternion_matrix(tf_q)[:3, :3].dot(direction)
        
        direction_symmetry = int(result.get('direction_symmetry', 1))
        best_direction_world = max(
            [transformations.rotation_matrix(i/direction_symmetry*2*math.pi, [0, 0, 1])[:3, :3].dot(direction_world) for i in xrange(direction_symmetry)],
            key=lambda direction: direction.dot(current._rot.dot(self._body_vec_align)))
        
        if current._rot.dot(self._body_vec_align).dot(best_direction_world) > \
                math.cos(math.radians(2)):
            return None
        
        print current.turn_vec_towards_rel(self._body_vec_align, best_direction_world)
        return current.turn_vec_towards_rel(self._body_vec_align, best_direction_world)

def select_first(targetreses, traj_start, tf):
    return targetreses[0]
    
def select_by_angle(direction_name):
    assert direction_name in ['left', 'right']
    def _(results, traj_start, (tf_p, tf_q)):
        def get_wantedness(result):
            direction = numpy.array(map(float, result['direction']))
            direction_world = transformations.quaternion_matrix(tf_q)[:3, :3].dot(direction)
            
            direction_symmetry = int(result.get('direction_symmetry', 1))
            best_direction_world = max(
                [transformations.rotation_matrix(i/direction_symmetry*2*math.pi, [0, 0, 1])[:3, :3].dot(direction_world) for i in xrange(direction_symmetry)],
                key=lambda direction: direction.dot(traj_start.forward_vector))
            print best_direction_world, traj_start.left_vector, best_direction_world.dot(traj_start.left_vector)*(1 if direction == 'left' else -1), 1 if direction == 'left' else -1
            return best_direction_world.dot(traj_start.left_vector)*(1 if direction_name == 'left' else -1)
        
        return max(results, key=get_wantedness)
    return _

