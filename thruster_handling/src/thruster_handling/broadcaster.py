import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3

from thruster_handling.msg import ThrusterCommand, ThrusterInfo

class ThrusterBroadcaster(object):
    def _command_callback(self, msg):
        self.command_callback(msg.force)
    
    def __init__(self, frame_id, id, lifetime, position, direction, min_force, max_force, torque_per_force, command_callback):
        self.frame_id = frame_id
        self.id = id
        self.lifetime = lifetime
        self.position = position
        self.direction = direction
        self.min_force = min_force
        self.max_force = max_force
        self.torque_per_force = torque_per_force
        self.command_callback = command_callback
        
        self.pub = rospy.Publisher('thrusters/info', ThrusterInfo)
        self._command_sub = rospy.Subscriber('thrusters/command/' + id, ThrusterCommand, self._command_callback)
    
    def send(self, active=True):
        self.pub.publish(ThrusterInfo(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=self.frame_id,
            ),
            id=self.id,
            lifetime=self.lifetime,
            active=active,
            position=Point(*self.position),
            direction=Vector3(*self.direction),
            min_force=self.min_force,
            max_force=self.max_force,
            torque_per_force=Vector3(*self.torque_per_force),
        ))
