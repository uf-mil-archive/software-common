import rospy
from std_msgs.msg import Header

from kill_handling.srv import SetKill
from kill_handling.msg import Kill

class KillBroadcaster(object):
    def __init__(self, id, description):
        self.id = id
        self.description = description

        self.set_kill = rospy.ServiceProxy('/set_kill', SetKill)
    
    def send(self, active):
        self.set_kill(Kill(
            header=Header(
                stamp=rospy.Time.now(),
            ),
            id=self.id,
            active=active,
            description=self.description,
        ), False)

    def clear(self):
        self.set_kill(Kill(
            header=Header(
                stamp=rospy.Time.now(),
            ),
            id=self.id,
        ), True)
        
