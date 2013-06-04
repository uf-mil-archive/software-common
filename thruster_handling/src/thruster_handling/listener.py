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

        self._command_pubs = {}

    def get_thrusters(self):
        t = rospy.Time.now()
        return [thruster for thruster in self._thruster_cache.values() if thruster.header.stamp + thruster.lifetime >= t and thruster.active]

    def get_thruster(self, id):
        return self._thruster_cache[id]

    def send_command(self, id, force):
        if id not in self._command_pubs:
            self._command_pubs[id] = rospy.Publisher('thrusters/command/' + id, ThrusterCommand)
        self._command_pubs[id].publish(ThrusterCommand(
            force=force,
        ))

    def unregister(self):
        self._sub.unregister()
        for pub in self._command_pubs.values():
            pub.unregister()
