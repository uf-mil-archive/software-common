import rospy

from std_msgs.msg import Header

from thruster_handling.msg import ThrusterCommand, ThrusterInfo


class ThrusterListener(object):
    def _thrusterinfo_callback(self, msg):
        if msg.id in self._thruster_cache and msg.header.stamp < self._thruster_cache[msg.id].header.stamp:
            return # this is older than current info
        
        self._thruster_cache[msg.id] = msg
    
    def __init__(self):
        self._thruster_cache = {}
        self._sub = rospy.Subscriber('thrusters/info', ThrusterInfo, self._thrusterinfo_callback)
        
        self._command_pub = rospy.Publisher('thrusters/command', ThrusterCommand)
    
    def get_thrusters(self):
        t = rospy.Time.now()
        return [thruster for thruster in self._thruster_cache.values() if thruster.header.stamp + thruster.lifetime >= t and thruster.active]
    
    def send_command(self, stamp, frame_id, id, force):
        self._command_pub.publish(ThrusterCommand(
            header=Header(
                stamp=stamp,
                frame_id=frame_id,
            ),
            id=id,
            force=force,
        ))
