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
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL,QTimer
from python_qt_binding.QtGui import QWidget, QPushButton,QCheckBox,QListWidget,QLineEdit

uipath = os.path.dirname(os.path.realpath(__file__))

global position
position = [0,0,0]

def pos_callback(msg):
        global position 
        position = [msg.x,msg.y,msg.z]

class GPSPlugin(Plugin):
    def __init__(self, context):
        super(GPSPlugin, self).__init__(context)
        self.setObjectName('GPS')

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'gps.ui'), self._widget)
        context.add_widget(self._widget)

        self.waypoint_latlong = rospy.Publisher('/gps_latlong_waypoint',NavSatFix)
        self.waypoint_ecef = rospy.Publisher('/gps_ecef_waypoint',PointStamped)
        rospy.Subscriber('/gps_parser/pos',PointStamped,pos_callback)

        self._widget.findChild(QPushButton, 'record_entered_waypoint').clicked.connect(self._on_record_entered_clicked)
        self._widget.findChild(QPushButton, 'record_current_waypoint').clicked.connect(self._on_record_current_clicked)
        self._widget.findChild(QPushButton, 'publish_waypoint_list').clicked.connect(self._on_pub_list_clicked)
        self._widget.findChild(QPushButton, 'delete_').clicked.connect(self._on_delete_clicked)
                
    def _on_pub_list_clicked(self):  
        self.rec_waypoint = self._widget.findChild(QListWidget, 'waypoint_list').currentItem().text()
        self.list = self.rec_waypoint.split(',')
        if (len(self.list) == 4):
                self.lat = self.list[1]
                self.long = self.list[2]
                self.alt = self.list[3]
                self.waypoint_latlong.publish(NavSatFix(
	                header=Header(
		                frame_id='/latlong',
	                ),
	                latitude = float(self.lat),
                        longitude = float(self.long),
                        altitude = float(self.alt),      
	                ))
        else:
                self.x = self.list[2]
                self.y = self.list[3]
                self.z = self.list[4]
                self.waypoint_ecef.publish(PointStamped(
                                            header=Header(
                                                stamp = rospy.Time.now(),
                                                frame_id='/ecef',
                                            ),
                                            point=Point(float(self.x), float(self.y), float(self.z)),
                                        ))
                                    
    def _on_record_current_clicked(self):
        global position
        self.name = self._widget.findChild(QLineEdit, 'waypoint_name').displayText()
        self._widget.findChild(QLineEdit, 'waypoint_name').clear()
        self._widget.findChild(QListWidget, 'waypoint_list').addItem(str('(ECEF)'+','+self.name+','+str(position[0])+','+str(position[1])+','+str(position[2])))

    def _on_record_entered_clicked(self):
        self.lat = self._widget.findChild(QLineEdit, 'lat_in').displayText()
        self.long = self._widget.findChild(QLineEdit, 'long_in').displayText()
        self.alt = self._widget.findChild(QLineEdit, 'alt_in').displayText()
        self.name = self._widget.findChild(QLineEdit, 'waypoint_name').displayText()  
        self._widget.findChild(QLineEdit, 'lat_in').clear()
        self._widget.findChild(QLineEdit, 'long_in').clear()
        self._widget.findChild(QLineEdit, 'alt_in').clear()
        self._widget.findChild(QLineEdit, 'waypoint_name').clear()
        
        self._widget.findChild(QListWidget, 'waypoint_list').addItem(str(self.name+','+self.lat+','+self.long+','+self.alt))

    def _on_delete_clicked(self):
       for SelectedItem in self._widget.findChild(QListWidget, 'waypoint_list').selectedItems():
               self._widget.findChild(QListWidget, 'waypoint_list').takeItem(self._widget.findChild(QListWidget, 'waypoint_list').row(SelectedItem))
        

    
    
   
