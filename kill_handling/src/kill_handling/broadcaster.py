import rospy
from std_msgs.msg import Header

from kill_handling.msg import Kill

class KillBroadcaster(object):
    def __init__(self, id, lifetime, description):
        self.id = id
        self.lifetime = lifetime
        self.description = description
        
        self.pub = rospy.Publisher('/kill', Kill)
    
    def send(self, active):
        self.pub.publish(Kill(
            header=Header(
                stamp=rospy.Time.now(),
            ),
            id=self.id,
            lifetime=self.lifetime,
            active=active,
            description=self.description,
        ))
