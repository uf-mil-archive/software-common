from __future__ import division
import roslib
roslib.load_manifest('uf_rqt_plugins')

import os
import rospy

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL,QTimer
from python_qt_binding.QtGui import QWidget, QPushButton,QCheckBox,QListWidget,QLineEdit

uipath = os.path.dirname(os.path.realpath(__file__))

class GPSPlugin(Plugin):
    def __init__(self, context):
        super(GPSPlugin, self).__init__(context)
        self.setObjectName('GPS')

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'gps.ui'), self._widget)
        context.add_widget(self._widget)

        self.waypoint = rospy.Publisher('/gps_waypoint',NavSatFix)

        self._widget.findChild(QPushButton, 'publish_waypoint').clicked.connect(self._on_publish_clicked)
        self._widget.findChild(QPushButton, 'record_waypoint').clicked.connect(self._on_record_clicked)
       
                

    def _on_publish_clicked(self):
        self.lat = self._widget.findChild(QLineEdit, 'lat_in').displayText()
        self.long = self._widget.findChild(QLineEdit, 'long_in').displayText()
        self.alt = self._widget.findChild(QLineEdit, 'alt_in').displayText()
        self.waypoint.publish(NavSatFix(
	        header=Header(
		        frame_id='/world',
	        ),
	        latitude = float(self.lat),
                longitude = float(self.long),
                altitude = float(self.alt),      
	        ))

    def _on_record_clicked(self):
        pass  

    
    
   
