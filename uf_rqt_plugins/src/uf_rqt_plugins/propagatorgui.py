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

import os
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

uipath = os.path.dirname(os.path.realpath(__file__))

class PropaGatorGUI(Plugin):
    def __init__(self, contex):
        super(PropaGatorGUI, self).__init__(contex)

        # Assign a name
        self.setObjectName('PropaGatorGUI')

        # Create a widget
        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'propagatorgui.ui'), self._widget)
        self._widget.setObjectName('PropaGatorGUI')
        contex.add_widget(self._widget)


        