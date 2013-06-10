import rospy

from kill_handling.msg import Kill


class KillListener(object):
    def _killmsg_callback(self, msg):
        self._kill_cache[msg.id] = rospy.Time.now(), msg

        self._check_killed()

    def _check_killed(self):
        killed = self.get_killed()
        if killed and not self._previously_killed:
            self._killed_callback()
        elif not killed and self._previously_killed:
            self._unkilled_callback()

    def __init__(self, killed_callback=lambda: None,
            unkilled_callback=lambda: None):
        self._killed_callback = killed_callback
        self._unkilled_callback = unkilled_callback

        self._kill_cache = {}
        self._sub = rospy.Subscriber('/kill', Kill, self._killmsg_callback)
        self._previously_killed = False
        self._check_killed_timer = rospy.Timer(rospy.Duration(.1),
            lambda timerinfo: self._check_killed())

    def get_kills(self):
        now = rospy.Time.now()
        return [kill.description for t, kill in self._kill_cache.values()
            if t + kill.lifetime >= now and kill.active]
    def get_killed(self):
        return bool(self.get_kills())
    def get_all_kills(self):
        now = rospy.Time.now()
        return [kill for t, kill in self._kill_cache.values()
                if t + kill.lifetime >= now]
