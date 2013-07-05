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
from python_qt_binding.QtGui import QWidget, QPushButton,QCheckBox,QListWidget,QLineEdit,QProgressBar, QLCDNumber
from power_router.msg import prstatus
from power_router.srv import SetKill


uipath = os.path.dirname(os.path.realpath(__file__))

global computer_current, motor_current, temperature, battery
computer_current = 0
motor_current = 0
temperature = 0
battery = 0

def data_update(msg):
	global computer_current, motor_current, temperature, battery
	computer_current = msg.computercurrent
	motor_current = msg.motorcurrent
	temperature = msg.temperature
	battery = msg.battery



class Engineering_Plant(Plugin):
    def __init__(self, context):
        super(Engineering_Plant, self).__init__(context)
        self.setObjectName('Engineering_Plant')

        self._widget = QWidget()
        loadUi(os.path.join(uipath, 'engineering_plant.ui'), self._widget)
        context.add_widget(self._widget)

        self._widget.findChild(QPushButton, 'Kill').clicked.connect(self.Kill)
        self._widget.findChild(QPushButton, 'UnKill').clicked.connect(self.Un_kill)
	
	rospy.Subscriber('/power_router/status',prstatus,data_update)	
        self.killservice = rospy.ServiceProxy('/power_router/setkill', SetKill)
	
        self._update_timer = QTimer(self._widget)
        self._update_timer.timeout.connect(self._on_update)
        self._update_timer.start(1000)

      

    def _on_update(self):
	global computer_current, motor_current, temperature, battery
	compratio = int(float(computer_current/800.0)*100)
	self._widget.findChild(QProgressBar, 'ComputerCurrent').setValue(compratio)	
	motorratio = int(float(motor_current/9000.0)*100)
	self._widget.findChild(QProgressBar, 'MotorCurrent').setValue(motorratio)
	#tempratio = int(float(temperature/150)*100)
	self._widget.findChild(QLCDNumber, 'tempLCD').display(temperature)
	battratio = int(float(battery/100)*100)
	self._widget.findChild(QProgressBar, 'Battery').setValue(battratio)
	    
    def Kill(self):
		 self.killservice(True)
    def Un_kill(self):
	         self.killservice(False)
 

                
  
    
    
   
