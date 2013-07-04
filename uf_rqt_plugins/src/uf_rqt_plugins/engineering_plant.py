from __future__ import division
import roslib
roslib.load_manifest('uf_rqt_plugins')

import os
import rospy

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped,Point
from sensor_msgs.msg import NavSatFix
from qt_gui.plugin import Plugin
from rawgps_common.gps import ecef_from_latlongheight,enu_from_ecef
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL,QTimer,QSettings
from python_qt_binding.QtGui import QWidget, QPushButton,QCheckBox,QListWidget,QLineEdit

uipath = os.path.dirname(os.path.realpath(__file__))


class Engineering_Plant(Plugin):
    def __init__(self, context):
        super(Engineering_Plant, self).__init__(context)
        self.setObjectName('Engineering_Plant')

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'engineering_plant.ui'), self._widget)
        context.add_widget(self._widget)

        self._widget.findChild(QPushButton, 'Kill').clicked.connect(self.Kill)
        self._widget.findChild(QPushButton, 'Un-Kill').clicked.connect(self.Un_kill)

        
        self._update_timer = QTimer(self._widget)
        self._update_timer.timeout.connect(self._on_update)
        self._update_timer.start(1000)

      

    def _on_update(self):
		pass      

                
  
    
    
   
