import math

import actionlib

from uf_common.msg import MoveToAction
from uf_common.orientation_helpers import xyz_array, xyzw_array
from object_finder.msg import FindAction
import tf
from tf import transformations

class SharedActionClient(object):
    def __init__(self, *args):
        self._client = actionlib.SimpleActionClient(*args)
        self._client.wait_for_server()
        self._callback = None
        self._feedback = None

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        self._client.send_goal(goal,
                               done_cb=self._done_cb,
                               active_cb=self._active_cb,
                               feedback_cb=self._feedback_cb)
        self.set_callbacks(done_cb, active_cb, feedback_cb)

    def set_callbacks(self, done_cb=None, active_cb=None, feedback_cb=None):
        self._active_callback = active_cb
        self._done_callback = done_cb
        self._feedback_callback = feedback_cb

    def clear_callbacks(self):
        self.set_callbacks()

    def get_feedback(self):
        return self._feedback

    def _done_cb(self, state, result):
        if self._done_callback is not None:
            self._done_callback(state, result)

    def _active_cb(self):
        if self._active_callback is not None:
            self._active_callback()

    def _feedback_cb(self, feedback):
        self._feedback = feedback
        if self._feedback_callback is not None:
            self._feedback_callback(feedback)

class StateSharedHandles(dict):
    def __init__(self):
        self['find_forward'] = SharedActionClient('find_forward', FindAction)
        self['find_down'] = SharedActionClient('find_down', FindAction)
        self['moveto'] = SharedActionClient('moveto', MoveToAction)
        self['tf_listener'] = tf.TransformListener()

    def clear_callbacks(self):
        for handle in self.itervalues():
            if isinstance(handle, SharedActionClient):
                handle.clear_callbacks()

def _yaw_angle_between(qa, qb):
    yaw =  transformations.euler_from_quaternion(
        transformations.quaternion_multiply(
            qa, transformations.quaternion_conjugate(qb))
        )[2]
    if yaw > math.pi/2:
        yaw -= math.pi
    elif yaw < -math.pi/2:
        yaw += math.pi
    return yaw

def left_orientation_selector(targetreses, traj_start):
    return sorted(
        targetreses,
        key=lambda result: _yaw_angle_between(
            traj_start.orientation, xyzw_array(result.pose.orientation))
        )[0]

def right_orientation_selector(targetreses, traj_start):
    return sorted(
        targetreses,
        key=lambda result: _yaw_angle_between(
            traj_start.orientation, xyzw_array(result.pose.orientation)),
        reverse=True
        )[0]

def _body_y_position(pos, traj_start):
    R_nav2body = transformations.quaternion_matrix(
        transformations.quaternion_inverse(traj_start.orientation))[:3, :3]
    return R_nav2body.dot(pos)[1]

def right_position_selector(targetreses, traj_start):
    return sorted(
        targetreses,
        key=lambda result: _body_y_position(xyz_array(result.pose.position),
                                            traj_start)
        )[0]

def left_position_selector(targetreses, traj_start):
    return sorted(
        targetreses,
        key=lambda result: _body_y_position(xyz_array(result.pose.position),
                                            traj_start),
        reverse=True
        )[0]

