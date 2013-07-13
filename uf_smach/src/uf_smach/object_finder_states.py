import threading
import numpy

import rospy
import smach
import smach_ros
import actionlib

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3
import tf
from tf import transformations

from uf_common.orientation_helpers import PoseEditor, xyz_array, xyzw_array
from uf_common.msg import MoveToAction, MoveToGoal, PoseTwist, PoseTwistStamped
from object_finder.msg import FindAction, FindGoal, TargetDesc

class WaitForObjectsState(smach.State):
    def __init__(self, shared, action, targetdescs_cb=None, P_within_10cm_thresh=.85, timeout=60):
        smach.State.__init__(self, outcomes=['succeeded', 'timeout'])

        self._shared = shared
        self._targetdescs_cb = targetdescs_cb
        self._P_within_10cm_thresh = P_within_10cm_thresh
        self._timeout = rospy.Duration(timeout)
        self._cond = threading.Condition()
        self._found = False
        self._action = action

    def execute(self, userdata):
        self._found = False
        if self._targetdescs_cb is not None:
            goal = FindGoal()
            goal.header.frame_id = "/map"
            goal.targetdescs = self._targetdescs_cb()
            self._shared[self._action].send_goal(goal, feedback_cb=self._feedback_cb)
        else:
            self._shared[self._action].set_callbacks(feedback_cb=self._feedback_cb)

        end_time = rospy.Time.now() + self._timeout
        with self._cond:
            while not self._found and rospy.Time.now() < end_time:
                self._cond.wait(1)

        self._shared.clear_callbacks()
        return 'succeeded' if self._found else 'timeout'

    def _feedback_cb(self, feedback):
        print [result.P_within_10cm for result in feedback.targetreses]
        if all(result.P_within_10cm > self._P_within_10cm_thresh
               for result in feedback.targetreses):
            with self._cond:
                self._found = True
                self._cond.notify_all()

class BaseManeuverObjectState(smach.State):
    def __init__(self, shared, action, selector=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._shared = shared
        self._action = action
        self._selector = selector
        self._cond = threading.Condition()
        self._done = False
        self._failed = False
        self._fail_ctr = 0

    def execute(self, userdata):
        self._done = False
        self._failed = False
        self._failed_ctr = 0
        self._traj_start = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        self._shared[self._action].set_callbacks(
            feedback_cb=lambda feedback: self._feedback_cb(feedback, self._shared))

        with self._cond:
            while not self._done and not self._failed:
                self._cond.wait()
            self._shared.clear_callbacks()

        return 'succeeded' if self._done else 'failed'

    def _feedback_cb(self, feedback, shared):
        with self._cond:
            print [result.P_within_10cm for result in feedback.targetreses]
            good_results = [result for result in feedback.targetreses
                            if result.P_within_10cm_xy > .85]
            if len(good_results) == 0:
                self._fail_ctr += 1
                if self._fail_ctr > 10:
                    self._failed = True;
                    self._cond.notify_all()
                return
            else:
                self._fail_ctr = 0

            if self._selector is not None:
                result = self._selector(good_results, self._traj_start)
            else:
                result = good_results[0]
            target = self._get_target(result)
            shared['moveto'].send_goal(target.as_MoveToGoal(speed=.2, linear_tolerance=.03),
                                       done_cb=self._done_cb)

    def _done_cb(self, state, result):
        with self._cond:
            self._done = True
            self._cond.notify_all()

class ApproachObjectState(BaseManeuverObjectState):
    def __init__(self, shared, action, approach_frame, approach_dist, selector=None, marker=None):
        BaseManeuverObjectState.__init__(self, shared, action, selector)
        self._approach_dist = approach_dist
        self._marker = marker

        tf_listener = self._shared['tf_listener']
        tf_listener.waitForTransform('/base_link', approach_frame,
                                     rospy.Time(0), rospy.Duration(10))
        self._approach_pos, _ = tf_listener.lookupTransform('/base_link',
                                                            approach_frame,
                                                            rospy.Time(0))
        self._approach_pos = numpy.array(self._approach_pos)

    def _get_target(self, result):
        origin = PoseEditor.from_Pose(self._traj_start.frame_id, result.pose)
        markers = dict((markerpoint.name, xyz_array(markerpoint.position))
            for markerpoint in result.markers)
        pose = origin.relative(markers[self._marker]) \
            if self._marker is not None else origin
        
        target = self._traj_start
        #target = target.look_at_without_pitching(pose.position)
        target = target.set_position(pose.position)
        target = target.relative(-self._approach_pos) \
                       .backward(self._approach_dist)
        return target

class AlignObjectState(BaseManeuverObjectState):
    def _get_target(self, result):
        dest_pos = xyz_array(result.pose.position)
        dest_orientation = xyzw_array(result.pose.orientation)

        # Rotate dest_orientation by 180 degrees if we're about to turn more than 90
        if numpy.dot(self._traj_start.orientation, dest_orientation)**2 < .5:
            dest_orientation = transformations.quaternion_multiply(dest_orientation,
                                                                   [0, 0, 1, 0])

        return self._traj_start.set_position(dest_pos) \
                               .set_orientation(dest_orientation) \
                               .height(self._traj_start.position[2])
        return target
