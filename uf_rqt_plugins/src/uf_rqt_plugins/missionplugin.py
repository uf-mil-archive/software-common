from __future__ import division
import roslib
roslib.load_manifest('uf_rqt_plugins')

import os
import rospy
import actionlib

from uf_smach.msg import PlansStamped, PlanEntry, RunMissionsAction, RunMissionsGoal
from uf_smach.srv import ModifyPlan, ModifyPlanRequest

import rospy
from std_msgs.msg import Header
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal, QTimer
from python_qt_binding.QtGui import QWidget, QFrame, QComboBox, \
    QPushButton, QSpinBox, QTreeWidget, QTreeWidgetItem

uipath = os.path.dirname(os.path.realpath(__file__))

class MissionPlugin(Plugin):
    on_plans_changed = pyqtSignal(PlansStamped)
    
    def __init__(self, context):
        Plugin.__init__(self, context)
        self.setObjectName('MissionPlugin')

        self._plans = None
        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'missionplugin.ui'), self._widget)
        context.add_widget(self._widget)

        self._widget.findChild(QPushButton, 'addButton').clicked.connect(self._on_add)
        self._widget.findChild(QPushButton, 'removeButton').clicked.connect(self._on_remove)
        self._widget.findChild(QPushButton, 'startButton').clicked.connect(self._on_start)
        
        self.on_plans_changed.connect(self._on_plans)
        self._plans_sub = rospy.Subscriber('mission/plans', PlansStamped,
                                           lambda msg: self.on_plans_changed.emit(msg))
        self._modify_srv = rospy.ServiceProxy('mission/modify_plan', ModifyPlan)

    def shutdown_plugin(self):
        self._plans_sub.unregister()

    def _on_add(self, event):
        selected_items = self._widget.findChild(QTreeWidget, 'missionTreeWidget').selectedItems()
        if len(selected_items) == 0:
            return
        item = selected_items[0]
        if item.parent() is not None:
            plan = item.parent().text(0)
            pos = item.parent().indexOfChild(item)+1
        else:
            plan = item.text(0)
            pos = 0

        contigency = self._widget.findChild(QComboBox, 'contigencyCombo').currentText()
        if contigency == 'none':
            contigency = ''
            
        entry = PlanEntry(
            mission=self._widget.findChild(QComboBox, 'missionCombo').currentText(),
            timeout=rospy.Duration(int(self._widget.findChild(QSpinBox, 'timeoutSpin').value())),
            contigency_plan=contigency,
            path=self._widget.findChild(QComboBox, 'pathCombo').currentText())
        self._modify_srv(plan, ModifyPlanRequest.INSERT, pos, entry)

    def _on_remove(self, event):
        selected_items = self._widget.findChild(QTreeWidget, 'missionTreeWidget').selectedItems()
        if len(selected_items) == 0:
            return
        item = selected_items[0]
        if item.parent() is None:
            return
        plan = item.parent().text(0)
        pos = item.parent().indexOfChild(item)
        self._modify_srv(plan, ModifyPlanRequest.REMOVE, pos, PlanEntry()) 

    def _on_start(self, event):
        self._run_action = actionlib.SimpleActionClient('mission/run', RunMissionsAction)
        self._run_action.wait_for_server()
        self._run_action.send_goal(RunMissionsGoal())
        
    def _on_plans(self, msg):
        if self._plans == msg.plans:
            return
        self._plans = msg.plans

        mission_tree = self._widget.findChild(QTreeWidget, 'missionTreeWidget')
        mission_tree.clear()
        contigency_combo = self._widget.findChild(QComboBox, 'contigencyCombo')
        contigency_combo.clear()

        for plan in self._plans:
            item = QTreeWidgetItem([plan.name])
            for entry in plan.entries:
                contigency = entry.contigency_plan
                if len(contigency) == 0:
                    contigency = 'none'
                item.addChild(QTreeWidgetItem([entry.mission, str(entry.timeout.secs),
                                              contigency, entry.path]))
            mission_tree.addTopLevelItem(item)
            item.setExpanded(True)
            contigency_combo.addItem(plan.name)
        contigency_combo.addItem('none')
        
        mission_combo = self._widget.findChild(QComboBox, 'missionCombo')
        mission_combo.clear()
        mission_combo.addItems(msg.available_missions)
