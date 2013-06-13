import rospy

from std_msgs.msg import Header

from thruster_handling.msg import ThrusterCommand, ThrusterInfo

class ThrusterListener(object):
    def _thrusterinfo_callback(self, msg):
        if msg.id in self._thruster_cache and msg.header.stamp < self._thruster_cache[msg.id][0].header.stamp:
            return # this is older than current info

        self._thruster_cache[msg.id] = (msg, rospy.Time.now())

    def _thrustercommand_callback(self, id, msg):
        self._command_cache[id] = msg

    def __init__(self):
        self._thruster_cache = {}
        self._sub = rospy.Subscriber('thrusters/info', ThrusterInfo, self._thrusterinfo_callback)

        self._command_pubs = {}

        self._command_cache = {}
        self._command_subs = {}

    def get_thrusters(self):
        t = rospy.Time.now()
        return [thruster for thruster, received in self._thruster_cache.values() if received + thruster.lifetime >= t and thruster.active]

    def get_thruster(self, id):
        return self._thruster_cache[id][0]

    def send_command(self, id, force):
        if id not in self._command_pubs:
            self._command_pubs[id] = rospy.Publisher('thrusters/command/' + id, ThrusterCommand)
        self._command_pubs[id].publish(ThrusterCommand(
            force=force,
        ))

    def get_command(self, id):
        if id not in self._command_subs:
            self._command_subs[id] = rospy.Subscriber('thrusters/command/' + id, ThrusterCommand,
                                                      lambda command: self._thrustercommand_callback(id, command))
        return self._command_cache.get(id)

    def unregister(self):
        self._sub.unregister()
        for pub in self._command_pubs.values():
            pub.unregister()
        for sub in self._command_subs.values():
            sub.unregister()
