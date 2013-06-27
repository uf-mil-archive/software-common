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
from python_qt_binding.QtCore import SIGNAL,QTimer
from python_qt_binding.QtGui import QWidget, QPushButton,QCheckBox,QListWidget,QLineEdit

uipath = os.path.dirname(os.path.realpath(__file__))

global position,tasks
position = [0,0,0]
rings_latlong = [1.0,2.0,3.0]
buoys_latlong = [1.0,2.0,3.0]
button_latlong = [1.0,2.0,3.0]
spock_latlong = [1.0,2.0,3.0]


tasks = [['rings',ecef_from_latlongheight(rings_latlong[0],rings_latlong[1],rings_latlong[2])],
         ['buoys',ecef_from_latlongheight(buoys_latlong[0],buoys_latlong[1],buoys_latlong[2])],
         ['button',ecef_from_latlongheight(button_latlong[0],button_latlong[1],button_latlong[2])],
         ['spock',ecef_from_latlongheight(spock_latlong[0],spock_latlong[1],spock_latlong[2])]]

def pos_callback(msg):
        global position 
        position = [msg.point.x,msg.point.y,msg.point.z]


class GPSPlugin(Plugin):
    def __init__(self, context):
        super(GPSPlugin, self).__init__(context)
        self.setObjectName('GPS')

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'gps.ui'), self._widget)
        context.add_widget(self._widget)

        self.waypoint_ecef = rospy.Publisher('/gps_ecef_waypoint',PointStamped)
        self.tasks = rospy.Publisher('/task_waypoints',PointStamped)
        rospy.Subscriber('/gps2_parser/pos',PointStamped,pos_callback)

        self._widget.findChild(QPushButton, 'record_entered_waypoint').clicked.connect(self._on_record_entered_clicked)
        self._widget.findChild(QPushButton, 'record_current_waypoint').clicked.connect(self._on_record_current_clicked)
        self._widget.findChild(QPushButton, 'publish_waypoint_list').clicked.connect(self._on_pub_list_clicked)
        self._widget.findChild(QPushButton, 'delete_').clicked.connect(self._on_delete_clicked)
        
        self._update_timer = QTimer(self._widget)
        self._update_timer.timeout.connect(self._on_update)
        self._update_timer.start(1000)

        global tasks
        for i in tasks:
                self._widget.findChild(QListWidget, 'waypoint_list').addItem(str(i[0])+','+str(i[1][0])+','+str(i[1][1])+','+str(i[1][2]))

    def _on_update(self):
        global tasks
        for i in tasks: 
                self.tasks.publish(PointStamped(
                                    header=Header(
                                        stamp = rospy.Time.now(),
                                        frame_id=i[0],
                                    ),
                                    point=Point(i[1][0], i[1][1], i[1][2]),
                                ))        

                
    def _on_pub_list_clicked(self):  
        self.rec_waypoint = self._widget.findChild(QListWidget, 'waypoint_list').currentItem().text()
        self.list = self.rec_waypoint.split(',')
   
        self.x = self.list[1]
        self.y = self.list[2]
        self.z = self.list[3]
        self.waypoint_ecef.publish(PointStamped(
                                    header=Header(
                                        stamp = rospy.Time.now(),
                                        frame_id='/ecef',
                                    ),
                                    point=Point(float(self.x), float(self.y), float(self.z)),
                                ))
                            
    def _on_record_current_clicked(self):
        global position,tasks
        self.name = self._widget.findChild(QLineEdit, 'waypoint_name').displayText()
        self._widget.findChild(QLineEdit, 'waypoint_name').clear()
        self._widget.findChild(QListWidget, 'waypoint_list').addItem(str(self.name)+','+str(position[0])+','+str(position[1])+','+str(position[2]))

        if str(self.name) in ['rings','buoys','button']:
                for i in tasks:
                        if (i[0] == str(self.name)):
                                tasks.remove(i)
                tasks.append([str(self.name),[float(position[0]),float(position[1]),float(position[2])]])

    def _on_record_entered_clicked(self):
        self.lat = self._widget.findChild(QLineEdit, 'lat_in').displayText()
        self.long = self._widget.findChild(QLineEdit, 'long_in').displayText()
        self.alt = self._widget.findChild(QLineEdit, 'alt_in').displayText()
        self.name = self._widget.findChild(QLineEdit, 'waypoint_name').displayText()  
        self._widget.findChild(QLineEdit, 'lat_in').clear()
        self._widget.findChild(QLineEdit, 'long_in').clear()
        self._widget.findChild(QLineEdit, 'alt_in').clear()
        self._widget.findChild(QLineEdit, 'waypoint_name').clear()

        ecef = ecef_from_latlongheight(float(self.lat), float(self.long), float(self.alt))
        
        self._widget.findChild(QListWidget, 'waypoint_list').addItem(str(self.name)+','+str(ecef[0])+','+str(ecef[1])+','+str(ecef[2]))

        global tasks
        if str(self.name) in ['rings','buoys','button']:
                for i in tasks:
                        if (i[0] == str(self.name)):
                                tasks.remove(i)
                tasks.append([str(self.name),[float(ecef[0]),float(ecef[1]),float(ecef[2])]])

    def _on_delete_clicked(self):
        self.rec_waypoint = self._widget.findChild(QListWidget, 'waypoint_list').currentItem().text()
        self.list = self.rec_waypoint.split(',')
   
        self.name = self.list[0]
        for i in tasks:
                        if (i[0] == str(self.name)):
                                tasks.remove(i)
        for SelectedItem in self._widget.findChild(QListWidget, 'waypoint_list').selectedItems():
               self._widget.findChild(QListWidget, 'waypoint_list').takeItem(self._widget.findChild(QListWidget, 'waypoint_list').row(SelectedItem))
        

    
    
   
