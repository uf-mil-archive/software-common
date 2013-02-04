import rospy

from kill_handling.msg import Kill


class KillListener(object):
    def _killmsg_callback(self, msg):
        if msg.id in self._kill_cache and msg.header.stamp < self._kill_cache[msg.id].header.stamp:
            return # this is older than current info
        
        killed_before = self.get_killed()
        
        self._kill_cache[msg.id] = msg
        
        self._check_killed()
    
    def _check_killed(self):
        killed = self.get_killed()
        if killed and not self._previously_killed:
            self._killed_callback()
        elif not killed and self._previously_killed:
            self._unkilled_callback()
    
    def __init__(self, killed_callback=lambda: None, unkilled_callback=lambda: None):
        self._killed_callback = killed_callback
        self._unkilled_callback = unkilled_callback
        
        self._kill_cache = {}
        self._sub = rospy.Subscriber('/kill', Kill, self._killmsg_callback)
        self._previously_killed = False
        self._check_killed_timer = rospy.Timer(rospy.Duration(.1), lambda timerinfo: self._check_killed())
    
    def get_kills(self):
        t = rospy.Time.now()
        return [kill.description for kill in self._kill_cache.values() if kill.header.stamp + kill.lifetime >= t and kill.active]
    def get_killed(self):
        return bool(self.get_kills())
