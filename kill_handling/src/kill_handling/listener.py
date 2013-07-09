import rospy

from kill_handling.msg import KillsStamped

class KillListener(object):
    def _killmsg_callback(self, msg):
        self._kills = msg.kills

        self._check_killed()

    def _check_killed(self):
        killed = self.get_killed()
        if killed and not self._previously_killed:
            self._killed_callback()
        elif not killed and self._previously_killed:
            self._unkilled_callback()
        self._previously_killed = killed

    def __init__(self, killed_callback=lambda: None,
            unkilled_callback=lambda: None):
        self._killed_callback = killed_callback
        self._unkilled_callback = unkilled_callback

        self._kills = None
        self._sub = rospy.Subscriber('/kill', KillsStamped, self._killmsg_callback)
        self._previously_killed = False

    def get_kills(self):
        if self._kills is not None:
            return [kill.description for kill in self._kills if kill.active]
        else:
            return []

    def get_killed(self):
        if self._kills is not None:
            return len(self.get_kills()) > 0
        else:
            return True

    def get_all_kills(self):
        return self._kills
