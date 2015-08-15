# propagator gui
#
# The propagator gui is a rqt plugin that displays the status of the boat and allows interaction
#
# Status displays:
#   * Kill 
#   * RC status 
#   * Floating status
#   * Obstical display
#       * Raw lidar data
#       * Buoys
#       * Gates
#   * System status
#       * Odom
#       * Servos
#       * Thrusters
#       * Lidar
#
# Interactions:
#   * Kill
#   * RC/autonomus mode switch
#   * WASD waypoint control
#   * Low level path planner mode drop down
#   * Lidar angle mode control
#   * Float
#
# Bugs:
#   * (Solved)memory errors occur when using python threads with ROS threads
#       * See _odom_callback and http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin

import roslib
roslib.load_manifest('uf_rqt_plugins')
from kill_handling.listener import KillListener
from kill_handling.broadcaster import KillBroadcaster
from nav_msgs.msg import Odometry
from uf_common.orientation_helpers import quat_to_rotvec, xyzw_array
import rospy

import os
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QPushButton, QLabel, QPixmap
from python_qt_binding.QtCore import QTimer, Qt, QThread, QObject, Signal


cwd = os.path.dirname(os.path.realpath(__file__))

class PropaGatorGUI(Plugin):
    odom_update_signal = Signal()

    def __init__(self, contex):
        super(PropaGatorGUI, self).__init__(contex)

        # Assign a name
        self.setObjectName('PropaGatorGUI')

        # Create a widget
        self._widget = QWidget()
        loadUi(os.path.join(cwd, 'propagatorgui.ui'), self._widget)
        self._widget.setObjectName('PropaGatorGUI')
        contex.add_widget(self._widget)

        # Grab all the children from the widget
        self._kill_label = self._widget.findChild(QLabel, 'kill_label')
        self._float_label = self._widget.findChild(QLabel, 'float_label')
        self._autonomous_label = self._widget.findChild(QLabel, 'autonomous_label')
        self._odom_x_label = self._widget.findChild(QLabel, 'odom_x_label')
        self._odom_y_label = self._widget.findChild(QLabel, 'odom_y_label')
        self._odom_yaw_label = self._widget.findChild(QLabel, 'odom_yaw_label')
        self._odom_d_x_label = self._widget.findChild(QLabel, 'odom_d_x_label')
        self._odom_d_y_label = self._widget.findChild(QLabel, 'odom_d_y_label')
        self._odom_d_yaw_label = self._widget.findChild(QLabel, 'odom_d_yaw_label')

        self._kill_push_btn = self._widget.findChild(QPushButton, 'kill_push_btn')

        # Load images
        self._green_indicator = QPixmap(os.path.join(cwd, 'green_indicator.png'))
        self._red_indicator = QPixmap(os.path.join(cwd, 'red_indicator.png'))

        # Set up ROS interfaces
        self._kill_listener = KillListener()
        self._kill_broadcaster = KillBroadcaster(id = 'PropaGator GUI', 
            description = 'PropaGator GUI kill')
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)

        # Connect push buttons
        self._kill_push_btn.toggled.connect(self._on_kill_push_btn_toggle)

        # Connect other signals
        self.odom_update_signal.connect(self._odom_update, Qt.BlockingQueuedConnection)

        # Set up update timer at 10Hz
        # A Qt timer is used instead of a ros timer since Qt components are updated
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._onUpdate)
        self.update_timer.start(100)

    # Everything needs to be turned off here
    def shutdown_plugin(self):
        self.update_timer.stop()
        self._odom_sub.unregister()
        del self._odom_sub
        # Kill broadcaster is not cleared, the user should unkill before closing the GUI
        del self._kill_broadcaster
        del self._kill_listener

    # Subscriber callbacks
    # Since this is in a different thread it is possible and likely that
        #   the drawing thread will try and draw while the text is being changed
        #   this causes all kinds of mahem such as segmentation faults, double free, ...
        #   To prevent this from hapening this thread emits a Qt signal which is set up
        #   to block the main thread as described here http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
    def _odom_callback(self, msg):
        self.last_odom_msg = msg
        self.odom_update_signal.emit()

    def _odom_update(self):
        pos = self.last_odom_msg.pose.pose.position
        yaw = quat_to_rotvec(xyzw_array(self.last_odom_msg.pose.pose.orientation))[2]
        vel = self.last_odom_msg.twist.twist.linear
        dYaw = self.last_odom_msg.twist.twist.angular.z

        self._odom_x_label.setText('X: %3.3f' % pos.x)
        self._odom_y_label.setText('Y: %3.3f' % pos.y)
        self._odom_yaw_label.setText('Yaw: %3.3f' % yaw)
        self._odom_d_x_label.setText('dX: %3.3f' % vel.x)
        self._odom_d_y_label.setText('dY: %3.3f' % vel.y)
        self._odom_d_yaw_label.setText('dYaw: %3.3f' % dYaw)

    # Push btn callbacks
    def _on_kill_push_btn_toggle(self, checked):
        if checked:
            self._kill_broadcaster.send(True)
        else:
            self._kill_broadcaster.send(False)

    # Update functions
    def _updateStatus(self):
        # Check if killed
        if self._kill_listener.get_killed():
            self._kill_label.setPixmap(self._red_indicator)
        else:
            self._kill_label.setPixmap(self._green_indicator)

        # Check float status
        self._float_label.setPixmap(self._green_indicator)

        # Check if in autonomous or RC
        self._autonomous_label.setPixmap(self._green_indicator)

    def _updateControl(self):
        pass

    def _updateLidar(self):
        pass

    def _onUpdate(self):
        self._updateStatus()
        self._updateControl()
        self._updateLidar()