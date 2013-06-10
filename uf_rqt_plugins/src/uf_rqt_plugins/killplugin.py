from __future__ import division
import roslib
roslib.load_manifest('uf_rqt_plugins')

import os
import rospy

from kill_handling.listener import KillListener
from kill_handling.broadcaster import KillBroadcaster

import rospy
from std_msgs.msg import Header
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL, QTimer
from python_qt_binding.QtGui import QWidget, QFrame, QLabel, QPushButton, QTableWidget, QTableWidgetItem, QColor

uipath = os.path.dirname(os.path.realpath(__file__))

class KillPlugin(Plugin):
    def __init__(self, context):
        super(KillPlugin, self).__init__(context)
        self.setObjectName('KillPlugin')

        self._listener = KillListener()
        self._broadcaster = KillBroadcaster(rospy.get_name(), rospy.Duration(.3), 'Software kill using KillPlugin')
        self._kill_active = False

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'killplugin.ui'), self._widget)
        context.add_widget(self._widget)

        self._widget.findChild(QTableWidget, 'killTable').setHorizontalHeaderLabels(["Name", "Status", "Description"])

        self._widget.findChild(QPushButton, 'killButton').clicked.connect(self._on_kill_clicked)
        self._widget.findChild(QPushButton, 'unkillButton').clicked.connect(self._on_unkill_clicked)
        self._widget.findChild(QPushButton, 'runButton').clicked.connect(self._on_run_clicked)

        self._update_timer = QTimer(self._widget)
        self._update_timer.timeout.connect(self._on_update)
        self._update_timer.start(100)

    def _on_update(self):
        self._update_kills()
        self._update_kill()

    def _on_kill_clicked(self):
        self._kill_active = True
        self._update_kill()

    def _on_unkill_clicked(self):
        self._kill_active = False
        self._update_kill()

    def _on_run_clicked(self):
        self._on_unkill_clicked()

    def _update_kills(self):
        new_kills = {kill.id: kill for kill in self._listener.get_all_kills()}

        table = self._widget.findChild(QTableWidget, 'killTable')

        row = 0
        while row < table.rowCount():
            id = table.item(row, 1)
            if id in new_kills:
                self._update_kill_entry(row, new_kills[id])
                del new_kills[id]
                row += 1
            else:
                table.removeRow(row)

        for kill in new_kills.values():
            row = table.rowCount()
            table.setRowCount(row+1)
            self._update_kill_entry(row, kill)

    def _update_kill_entry(self, row, kill):
        color = QColor(255, 255, 255) if not kill.active else QColor(255, 200, 200)
        self._update_item(row, 0, color, kill.id)
        self._update_item(row, 1, color, "Killed" if kill.active else "Unkilled")
        self._update_item(row, 2, color, kill.description)

    def _update_item(self, row, col, color, string):
        item = QTableWidgetItem(string)
        item.setBackground(color)
        self._widget.findChild(QTableWidget, 'killTable').setItem(row, col, item)

    def _update_kill(self):
        self._broadcaster.send(self._kill_active)

        other_kill_count = len([kill for kill in self._listener.get_all_kills()
                                if kill.id != rospy.get_name() and kill.active])
        self._widget.findChild(QPushButton, 'runButton').setVisible(other_kill_count == 0)
        self._widget.findChild(QPushButton, 'unkillButton').setVisible(other_kill_count > 0)

        status = 'Sub status: '
        if not self._listener.get_killed():
            status += '<span style="color:green">Running</span>'
        else:
            status += '<span style="color:red">Killed</span>'
        self._widget.findChild(QLabel, 'killStatusLabel').setText(status)
