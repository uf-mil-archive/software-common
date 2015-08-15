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

import roslib
roslib.load_manifest('uf_rqt_plugins')
from kill_handling.listener import KillListener

import os
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QPushButton, QLabel, QPixmap
from python_qt_binding.QtCore import SIGNAL, QTimer, Qt


cwd = os.path.dirname(os.path.realpath(__file__))

class PropaGatorGUI(Plugin):
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

        # Load images
        self._green_indicator = QPixmap(os.path.join(cwd, 'green_indicator.png'))
        self._red_indicator = QPixmap(os.path.join(cwd, 'red_indicator.png'))

        # Set up ROS interfaces
        self._kill_listener = KillListener(None, None)

        # Set up update timer at 10Hz
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._onUpdate)
        self.update_timer.start(100)

    def _onUpdate(self):
        # Check if killed
        if self._kill_listener.get_killed():
            self._kill_label.setPixmap(self._red_indicator)
        else:
            self._kill_label.setPixmap(self._green_indicator)


        self._float_label.setPixmap(self._green_indicator)
        self._autonomous_label.setPixmap(self._green_indicator)