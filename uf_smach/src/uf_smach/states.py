import threading
import numpy

import rospy
import smach
import smach_ros
import actionlib

from uf_common.orientation_helpers import PoseEditor, xyz_array
from uf_common.msg import MoveToAction, MoveToGoal, PoseTwist
from object_finder.msg import FindAction, FindGoal, TargetDesc
from nav_msgs.msg import Odometry
import tf

class WaypointState(smach_ros.SimpleActionState):
    def __init__(self, goal_func):
        self._goal_func = goal_func

        # Make sure pose_func is valid
        assert(isinstance(self._goal_func(PoseEditor('/test',
                                                     numpy.array([0, 0, 0]),
                                                     numpy.array([1, 0, 0, 0]))),
                          PoseEditor))

        smach_ros.SimpleActionState.__init__(
            self, 'moveto', MoveToAction, goal_cb=self._goal_cb)

    def _goal_cb(self, userdata, goal):
        current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        goal = self._goal_func(current)
        return goal.as_MoveToGoal()

class VelocityState(smach.State):
    def __init__(self, vel):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._vel = vel
        self._client = actionlib.SimpleActionClient('moveto', MoveToAction)
        self._client.wait_for_server()

    def execute(self, userdata):
        current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        self._client.send_goal(current.as_MoveToGoal(linear=self._vel))
        return 'succeeded'

class WaitForSingleObjectState(smach.State):
    def __init__(self, action, targetdesc, timeout=60):
        smach.State.__init__(self, outcomes=['succeeded', 'timeout'])
        self._timeout = rospy.Duration(timeout)
        self._targetdesc = targetdesc
        self._cond = threading.Condition()
        self._found = False

        self._client = actionlib.SimpleActionClient(action, FindAction)
        self._client.wait_for_server()

    def execute(self, userdata):
        goal = FindGoal()
        goal.header.frame_id = "/map"
        goal.targetdescs = [self._targetdesc]
        self._client.send_goal(goal, feedback_cb=self._feedback_cb)

        end_time = rospy.Time.now() + self._timeout
        with self._cond:
            while not self._found and rospy.Time.now() < end_time:
                self._cond.wait(1)

        self._client.stop_tracking_goal()
        return 'succeeded' if self._found else 'timeout'

    def _feedback_cb(self, feedback):
        if len(feedback.targetreses) == 0:
            return
        result = feedback.targetreses[0]
        if result.P > 75:
            with self._cond:
                self._found = True
                self._cond.notify_all()

class ApproachObjectState(smach.State):
    def __init__(self, action, targetdesc, approach_frame, approach_dist):
        smach.State.__init__(self, outcomes=['succeeded'])

        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform('/base_link', approach_frame,
                                     rospy.Time(0), rospy.Duration(1000000))
        self._approach_pos, _ = tf_listener.lookupTransform('/base_link',
                                                            approach_frame,
                                                            rospy.Time(0))
        self._approach_pos = numpy.array(self._approach_pos)
        self._approach_dist = approach_dist
        self._targetdesc = targetdesc

        self._cond = threading.Condition()
        self._odom_current = None
        self._odom_start = None
        self._target_pose = None
        self._done = False

        self._vision_client = actionlib.SimpleActionClient(action, FindAction)
        self._vision_client.wait_for_server()
        self._move_client = actionlib.SimpleActionClient('moveto', MoveToAction)
        self._move_client.wait_for_server()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odometry_cb)

    def execute(self, userdata):
        vision_goal = FindGoal()
        vision_goal.header.frame_id = "/map"
        vision_goal.targetdescs = [self._targetdesc]
        self._vision_client.send_goal(vision_goal, feedback_cb=self._feedback_cb)

        with self._cond:
            while True:
                self._cond.wait()
                if self._target_pose is None or self._odom_current is None:
                    continue
                err = numpy.linalg.norm(xyz_array(self._target_pose.position) -
                                        xyz_array(self._odom_current.pose.pose.position))
                if err > 0.05:
                    continue
                break

        self._vision_client.stop_tracking_goal()
        return 'succeeded'

    def _feedback_cb(self, feedback):
        if len(feedback.targetreses) == 0 or self._odom_current is None:
            return
        result = feedback.targetreses[0]

        target = PoseEditor.from_Odometry(self._odom_start)
        target = target.look_at_without_pitching(xyz_array(result.pose.position)) \
                       .set_position(xyz_array(result.pose.position))
        target = target.relative(-self._approach_pos) \
                       .backward(self._approach_dist)
        self._move_client.send_goal(target.as_MoveToGoal(speed=.5))
        with self._cond:
            self._target_pose = target.as_Pose()
            self._cond.notify_all()

    def _odometry_cb(self, odometry):
        with self._cond:
            self._odom_current = odometry
            if self._odom_start is None:
                self._odom_start = odometry
            self._cond.notify_all()

