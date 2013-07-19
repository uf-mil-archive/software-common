from __future__ import division
import roslib
roslib.load_manifest('uf_rqt_plugins')

import os
import rospy

from actuator_driver.srv import PulseValve, SetValve
from actuator_driver.msg import Switches

import rospy
from std_msgs.msg import Header
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal, QTimer
from python_qt_binding.QtGui import QWidget, QFrame, QLabel, QPushButton, QColor

uipath = os.path.dirname(os.path.realpath(__file__))

# TODO put this somewhere?
VALVE_DROPPER = 0
VALVE_GRABBER_CLOSE = 1
VALVE_GRABBER_OPEN = 2
VALVE_SHOOTER_RIGHT = 3
VALVE_GAS_POWERED_STICK = 4
VALVE_SHOOTER_LEFT = 5

class ActuatorPlugin(Plugin):
    switches_changed = pyqtSignal(Switches)

    def __init__(self, context):
        super(ActuatorPlugin, self).__init__(context)
        self.setObjectName('ActuatorPlugin')

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'actuatorplugin.ui'), self._widget)
        context.add_widget(self._widget)

        self._setvalve = rospy.ServiceProxy('actuator_driver/set_valve', SetValve)
        self._pulsevalve = rospy.ServiceProxy('actuator_driver/pulse_valve', PulseValve)
        self._switches_sub = rospy.Subscriber('actuator_driver/switches', Switches,
                                              lambda msg: self.switches_changed.emit(msg))

        self.switches_changed.connect(self._on_switches_changed)
        self._widget.findChild(QPushButton, 'shootLeftButton').clicked.connect(
            lambda: self._pulsevalve(VALVE_SHOOTER_LEFT, rospy.Duration(.3)))
        self._widget.findChild(QPushButton, 'shootRightButton').clicked.connect(
            lambda: self._pulsevalve(VALVE_SHOOTER_RIGHT, rospy.Duration(.3)))
        self._widget.findChild(QPushButton, 'extendButton').clicked.connect(
            lambda: self._setvalve(VALVE_GAS_POWERED_STICK, True))
        self._widget.findChild(QPushButton, 'retractButton').clicked.connect(
            lambda: self._setvalve(VALVE_GAS_POWERED_STICK, False))
        self._widget.findChild(QPushButton, 'openButton').clicked.connect(self._open_grabber)
        self._widget.findChild(QPushButton, 'closeButton').clicked.connect(self._close_grabber)
        self._widget.findChild(QPushButton, 'dropButton').clicked.connect(
            lambda: self._pulsevalve(VALVE_DROPPER, rospy.Duration(.5)))
        self._widget.findChild(QPushButton, 'shutAllValvesButton').clicked.connect(self._shut_valves)
            
    def shutdown_plugin(self):
        self._switches_sub.unregister()

    def _open_grabber(self):
        self._setvalve(VALVE_GRABBER_CLOSE, False)
        self._setvalve(VALVE_GRABBER_OPEN, True)

    def _close_grabber(self):
        self._setvalve(VALVE_GRABBER_OPEN, False)
        self._setvalve(VALVE_GRABBER_CLOSE, True)

    def _shut_valves(self):
        for i in xrange(6):
            self._setvalve(i, False)

    def _on_switches_changed(self, msg):
        if all(msg.pressed):
            msg = '<span style="color: red">Pressed</span>'
        elif any(msg.pressed):
            msg = '<span style="color: yellow">Single pressed</span>'
        else:
            msg = '<span style="color: green">Released</span>'
        self._widget.findChild(QLabel, 'switchLabel').setText(msg)
