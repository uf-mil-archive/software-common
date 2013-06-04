from __future__ import division
import roslib
roslib.load_manifest('uf_rqt_plugins')

import os
import rospy

from thruster_handling.listener import ThrusterListener

import rospy
from std_msgs.msg import Header
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL, QTimer
from python_qt_binding.QtGui import QWidget, QFrame, QLabel, QPushButton, QSlider, QCheckBox

uipath = os.path.dirname(os.path.realpath(__file__))

class ThrusterPlugin(Plugin):
    def __init__(self, context):
        super(ThrusterPlugin, self).__init__(context)
        self.setObjectName('ThrusterPlugin')
        self._listener = ThrusterListener()

        self._thruster_widgets = {}

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'thrusterplugin.ui'), self._widget)
        context.add_widget(self._widget)
        self._widget.findChild(QPushButton, 'stopAllButton').clicked.connect(self.stop_all)

        self._update_timer = QTimer(self._widget)
        self._update_timer.timeout.connect(self._update)
        self._update_timer.start(100)

    def _update(self):
        active_ids = set()
        for thruster_info in self._listener.get_thrusters():
            active_ids.add(thruster_info.id)
            if thruster_info.id not in self._thruster_widgets:
                self._add_thruster_widget(thruster_info.id)

        for id in self._thruster_widgets:
            if id not in active_ids:
                self._remove_thruster_widget(id)

    def _add_thruster_widget(self, id):
        self._widget.findChild(QLabel, 'noThrustersLabel').setVisible(False)

        thruster_widget = QWidget()
        thruster_frame = self._widget.findChild(QFrame, 'thrusterFrame')
        pos = sum(1 for existing_id in self._thruster_widgets if id > existing_id)
        thruster_frame.layout().insertWidget(pos, thruster_widget)
        loadUi(os.path.join(uipath, 'thruster.ui'), thruster_widget)

        thruster_widget.findChild(QLabel, 'thrusterLabel').setText(id)
        thruster_widget.findChild(QCheckBox, 'reverseCheckbox').stateChanged.connect(lambda: self._update_thruster(id))
        thruster_widget.findChild(QSlider, 'thrusterSlider').valueChanged.connect(lambda: self._update_thruster(id))
        thruster_widget.findChild(QPushButton, 'offButton').clicked.connect(lambda: self.stop_thruster(id))

        self._thruster_widgets[id] = thruster_widget
        return thruster_widget

    def _remove_thruster_widget(self, thruster):
        if thruster not in self._thruster_widgets:
            return

        self._thruster_widgets[thruster].deleteLater()
        del self._thruster_widgets[thruster]

        if len(self._thruster_widgets) == 0:
            self._widget.findChild(QLabel, 'noThrustersLabel').setVisible(True)

    def stop_all(self):
        for thruster_widget in self._thruster_widgets.values():
            thruster_widget.findChild(QSlider, 'thrusterSlider').setValue(0)

    def stop_thruster(self, name):
        self._thruster_widgets[name].findChild(QSlider, 'thrusterSlider').setValue(0)

    def _update_thruster(self, name):
        thruster_widget = self._thruster_widgets[name]
        slider_widget = thruster_widget.findChild(QSlider, 'thrusterSlider')
        reverse_widget = thruster_widget.findChild(QCheckBox, 'reverseCheckbox')

        effort = slider_widget.value()/100
        if not reverse_widget.isChecked():
            force = effort * self._listener.get_thruster(name).max_force
        else:
            force = effort * self._listener.get_thruster(name).min_force

        print force

        self._listener.send_command(name, force)

    def shutdown_plugin(self):
        self._listener.unregister()
