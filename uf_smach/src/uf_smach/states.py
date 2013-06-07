import smach
import smach_ros

from uf_common.orientation_helpers import PoseEditor
from uf_common.msg import MoveToAction, MoveToGoal, PoseTwist

class WaypointState(smach_ros.SimpleActionState):
    def __init__(self, cmd, *args):
        self._cmd_func = getattr(PoseEditor, cmd, None)
        if self._cmd_func is None:
            raise ValueError('Invalid PoseEditor command "%s"' % cmd)
        self._cmd_args = args

        smach_ros.SimpleActionState.__init__(
            self, 'moveto', MoveToAction, goal_cb=self._goal_cb)

    def _goal_cb(self, userdata, goal):
        current = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        goal = self._cmd_func(current, *self._cmd_args)
        return goal

