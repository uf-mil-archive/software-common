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
from rise_6dof.srv import SendConstantWrench
from indirect_kalman_6dof.srv import SetPosition

class WaypointState(smach.State):
    def __init__(self, shared, goal_func):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

        self._shared = shared
        self._goal_func = goal_func
        self._cond = threading.Condition()
        self._done = False

        # Make sure goal_func is valid
        assert(isinstance(self._goal_func(PoseEditor('/test',
                                                     numpy.array([0, 0, 0]),
                                                     numpy.array([1, 0, 0, 0]))),
                          PoseEditor))

    def execute(self, userdata):
        current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        goal = self._goal_func(current)
        self._shared['moveto'].send_goal(goal, done_cb=self._done_cb)

        with self._cond:
            while not self._done and not self.preempt_requested():
                self._cond.wait(0.1)

        self._shared.clear_callbacks()
        if self.preempt_requested():
            return 'preempted'
        return 'succeeded'

    def _done_cb(self, state, result):
        with self._cond:
            self._done = True
            self._cond.notify_all()

class VelocityState(smach.State):
    def __init__(self, shared, vel):
        smach.State.__init__(self, outcomes=['succeeded'])

        self._shared = shared
        self._vel = vel

    def execute(self, userdata):
        current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        self._shared['moveto'].send_goal(current.as_MoveToGoal(linear=self._vel))
        return 'succeeded'

class ServiceState(smach.State):
    def __init__(self, name, service_class, *args, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        while True:
            try:
                rospy.wait_for_service(name, timeout=3)
            except:
                import traceback; traceback.print_exc()
                print 'Waiting for service %s...' % (name,)
                import time; time.sleep(1) # rospy doesn't have WallDuration::sleep
            else:
                break
        self._serviceproxy = rospy.ServiceProxy(name, service_class)
        self.args = args
        self.kwargs = kwargs
    
    def execute(self, userdata):
        try:
            res = self._serviceproxy.call(*self.args, **self.kwargs)
        except rospy.ServiceException:
            import traceback; traceback.print_exc()
            return 'failed'
        return 'succeeded'

class OpenLoopState(smach.State):
    def __init__(self, shared, torque, time):
        smach.State.__init__(self, outcomes=['succeeded'])

        self._shared = shared
        self._torque = torque
        self._time = time

    def execute(self, userdata):
        initial_odom = rospy.wait_for_message('/imu_odom', Odometry)
        send_constant_wrench = rospy.ServiceProxy('/send_constant_wrench',
                                                  SendConstantWrench)
        set_position = rospy.ServiceProxy('/indirect_kalman_6dof/set_position',
                                          SetPosition)

        send_constant_wrench(Wrench(Vector3(0, 0, 0),
                                    Vector3(*self._torque)),
                             rospy.Duration(self._time))
        set_position(initial_odom.pose.pose.position)
        return 'succeeded'

class SleepState(smach.State):
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        
        self._duration = duration

    def execute(self, userdata):
        start = rospy.Time.now()
        while True:
            if self.preempt_requested():
                return 'preempted'
            if rospy.Time.now() - start >= self._duration:
                return 'succeeded'
            rospy.sleep(.1)
