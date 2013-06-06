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
        self._manual_control = False

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'thrusterplugin.ui'), self._widget)
        context.add_widget(self._widget)
        self._widget.findChild(QPushButton, 'stopAllButton').clicked.connect(self._on_stopall_clicked)
        self._widget.findChild(QCheckBox, 'manualControlCheckBox').clicked.connect(self._on_manualcontrol_clicked)

        self._update_timer = QTimer(self._widget)
        self._update_timer.timeout.connect(self._update)
        self._update_timer.start(100)

    def _update(self):
        active_ids = set()
        for thruster_info in self._listener.get_thrusters():
            active_ids.add(thruster_info.id)
            if thruster_info.id not in self._thruster_widgets:
                self._add_thruster_widget(thruster_info.id)

        for id in self._thruster_widgets.keys():
            if id not in active_ids:
                self._remove_thruster_widget(id)
            else:
                self._update_thruster(id)

    def _add_thruster_widget(self, id):
        self._widget.findChild(QLabel, 'noThrustersLabel').setVisible(False)

        thruster_widget = QWidget()
        thruster_frame = self._widget.findChild(QFrame, 'thrusterFrame')
        pos = sum(1 for existing_id in self._thruster_widgets if id > existing_id)
        thruster_frame.layout().insertWidget(pos, thruster_widget)
        loadUi(os.path.join(uipath, 'thruster.ui'), thruster_widget)

        thruster_widget.findChild(QLabel, 'thrusterLabel').setText(id)
        thruster_widget.findChild(QPushButton, 'offButton').clicked.connect(lambda: self._on_stop_clicked(id))

        self._thruster_widgets[id] = thruster_widget
        return thruster_widget

    def _remove_thruster_widget(self, thruster):
        if thruster not in self._thruster_widgets:
            return

        self._thruster_widgets[thruster].deleteLater()
        del self._thruster_widgets[thruster]

        if len(self._thruster_widgets) == 0:
            self._widget.findChild(QLabel, 'noThrustersLabel').setVisible(True)

    def _update_thrusters(self):
        for thruster in self._listener.get_thrusters():
            self._update_thruster(thruster)

    def _update_thruster(self, id):
        thruster = self._listener.get_thruster(id)
        thruster_widget = self._thruster_widgets[id]
        slider_widget = thruster_widget.findChild(QSlider, 'thrusterSlider')
        reverse_widget = thruster_widget.findChild(QCheckBox, 'reverseCheckbox')

        thruster_widget.findChild(QPushButton, 'offButton').setEnabled(self._manual_control)
        thruster_widget.findChild(QCheckBox, 'reverseCheckbox').setEnabled(self._manual_control)
        thruster_widget.findChild(QSlider, 'thrusterSlider').setEnabled(self._manual_control)

        if not self._manual_control:
            command = self._listener.get_command(id)
            if command is None:
                return
            if command.force > 0:
                effort = command.force / thruster.max_force
            else:
                effort = command.force / thruster.min_force
            slider_widget.setValue(abs(effort) * 100)
            reverse_widget.setChecked(effort < 0)
        else:
            effort = slider_widget.value()/100
            if not reverse_widget.isChecked():
                force = effort * thruster.max_force
            else:
                force = effort * thruster.min_force
            self._listener.send_command(id, force)

        if abs(effort) > .9:
            color = 'red'
        elif abs(effort) > .75:
            color = 'yellow'
        else:
            color = 'black'

        thruster_widget.findChild(QLabel, 'thrusterLabel').setText(
            '<span style="color: ' + color + '">' + id + '</span>')

    def _on_stopall_clicked(self):
        for thruster_widget in self._thruster_widgets.values():
            thruster_widget.findChild(QSlider, 'thrusterSlider').setValue(0)

    def _on_stop_clicked(self, id):
        self._thruster_widgets[id].findChild(QSlider, 'thrusterSlider').setValue(0)

    def _on_manualcontrol_clicked(self):
        self._manual_control = self._widget.findChild(QCheckBox, 'manualControlCheckBox').isChecked()

    def shutdown_plugin(self):
        self._listener.unregister()
