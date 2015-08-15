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
from kill_handling.broadcaster import KillBroadcaster

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

        self._kill_push_btn = self._widget.findChild(QPushButton, 'kill_push_btn')

        # Load images
        self._green_indicator = QPixmap(os.path.join(cwd, 'green_indicator.png'))
        self._red_indicator = QPixmap(os.path.join(cwd, 'red_indicator.png'))

        # Set up ROS interfaces
        self._kill_listener = KillListener()
        self._kill_broadcaster = KillBroadcaster(id = 'PropaGator GUI', 
            description = 'PropaGator GUI kill')

        # Connect push buttons
        self._kill_push_btn.toggled.connect(self._on_kill_push_btn_toggle)

        # Set up update timer at 10Hz
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._onUpdate)
        self.update_timer.start(100)


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