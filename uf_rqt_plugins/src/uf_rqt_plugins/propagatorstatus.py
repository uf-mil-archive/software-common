from __future__ import division
import roslib
roslib.load_manifest('uf_rqt_plugins')

import os
import rospy

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix,Imu,Image,LaserScan
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL,QTimer,Qt
from python_qt_binding.QtGui import QWidget, QPushButton,QCheckBox,QListWidget,QLineEdit,QListWidgetItem
from thruster_handling.msg import ThrusterInfo
from skytraq_driver.msg import SerialPacket

uipath = os.path.dirname(os.path.realpath(__file__))

global active,check
active = []
check = []

def kill(event):
        global active,check
        active = []
        for i in ['FR','FL','BR','BL','GPS','IMU','LIDAR','CAMERA']:
                if (i in check):
                        active.append(i)
        check = []

def motordriver_callback(msg):
        if (msg.id == 'FR'):
                active.append('FR')
                check.append('FR')
        elif(msg.id == 'FL'):
                active.append('FL')
                check.append('FL')
        elif(msg.id == 'BR'):
                active.append('BR')
                check.append('BR')
        elif(msg.id == 'BL'):
                active.append('BL')
                check.append('BL')
def gps_callback(msg):
        active.append('GPS')
        check.append('GPS')
def imu_callback(msg):
        active.append('IMU')
        check.append('IMU')
def lidar_callback(msg):
        active.append('LIDAR')
        check.append('LIDAR')
def camera_callback(msg):
        active.append('CAMERA')
        check.append('CAMERA')

class PropStatus(Plugin):
    def __init__(self, context):
        super(PropStatus, self).__init__(context)
        self.setObjectName('Status')

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'propagatorstatus.ui'), self._widget)
        context.add_widget(self._widget)
        
        rospy.Subscriber('/thrusters/info',ThrusterInfo,motordriver_callback)
        rospy.Subscriber('/skytraq_serial',SerialPacket,gps_callback)
        rospy.Subscriber('/imu/data_raw',Imu,imu_callback)
        rospy.Subscriber('/scan',LaserScan,lidar_callback)
        rospy.Subscriber('/mv_bluefox_camera_node/image_raw',Image,camera_callback)
        rospy.Timer(rospy.Duration(2),kill)

        self._update_timer = QTimer(self._widget)
        self._update_timer.timeout.connect(self._on_update)
        self._update_timer.start(100)

    def _on_update(self):
        self._widget.findChild(QListWidget, 'list').clear()
        
        for i in ['FR','FL','BR','BL','GPS','IMU','LIDAR','CAMERA']:
                pItem = QListWidgetItem(i);
                if i in active: 
                        pItem.setBackground(Qt.green); 
                else:
                        pItem.setBackground(Qt.red); 
                self._widget.findChild(QListWidget, 'list').addItem(pItem)
                
        
   

                
