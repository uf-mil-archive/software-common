# propagator gui
#
# The propagator gui is a rqt plugin that displays the status of the boat and allows interaction
#
# Status displays:
#   * Kill 
#   * RC status 
#   * Floating status
#   * Obstical display
#       * Raw lidar data
#       * Buoys
#       * Gates
#   * System status
#       * Odom
#       * Servos
#       * Thrusters
#       * Lidar
#
# Interactions:
#   * Kill
#   * RC/autonomus mode switch
#   * WASD waypoint control
#   * Low level path planner mode drop down
#   * Lidar angle mode control
#   * Float
#
# Bugs:
#   * (Solved)memory errors occur when using python threads with ROS threads
#       * See _odom_cb and http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin

import roslib
roslib.load_manifest('uf_rqt_plugins')
from kill_handling.listener import KillListener
from kill_handling.broadcaster import KillBroadcaster
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from controller.srv import FloatMode, request_controller, get_all_controllers
from uf_common.orientation_helpers import quat_to_rotvec, xyzw_array
import rospy

import os
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QPushButton, QLabel, QPixmap, QComboBox
from python_qt_binding.QtCore import QTimer, Qt, QThread, QObject, Signal


cwd = os.path.dirname(os.path.realpath(__file__))

class PropaGatorGUI(Plugin):
    def __init__(self, contex):
        super(PropaGatorGUI, self).__init__(contex)

        # Initilize variables
        self._float_status = False
        self.last_odom_msg = Odometry()
        self._current_controller = 'Controller: Unknown'
        self._controller = ''
        self._controllers = []

        # Assign a name
        self.setObjectName('PropaGatorGUI')

        # Create a widget
        self._widget = QWidget()
        loadUi(os.path.join(cwd, 'propagatorgui.ui'), self._widget)
        self._widget.setObjectName('PropaGatorGUI')
        contex.add_widget(self._widget)

        # Grab all the children from the widget
        self._kill_label = self._widget.findChild(QLabel, 'kill_label')
        self._float_label = self._widget.findChild(QLabel, 'float_label')
        self._controller_label = self._widget.findChild(QLabel, 'controller_label')
        self._odom_x_label = self._widget.findChild(QLabel, 'odom_x_label')
        self._odom_y_label = self._widget.findChild(QLabel, 'odom_y_label')
        self._odom_yaw_label = self._widget.findChild(QLabel, 'odom_yaw_label')
        self._odom_d_x_label = self._widget.findChild(QLabel, 'odom_d_x_label')
        self._odom_d_y_label = self._widget.findChild(QLabel, 'odom_d_y_label')
        self._odom_d_yaw_label = self._widget.findChild(QLabel, 'odom_d_yaw_label')
        self._placeholder_label = self._widget.findChild(QLabel, 'placeholder_label')

        self._kill_push_btn = self._widget.findChild(QPushButton, 'kill_push_btn')
        self._float_push_btn = self._widget.findChild(QPushButton, 'float_push_btn')
        self._controller_combo_box = self._widget.findChild(QComboBox, 'controller_combo_box')

        # Load images
        self._green_indicator = QPixmap(os.path.join(cwd, 'green_indicator.png'))
        self._red_indicator = QPixmap(os.path.join(cwd, 'red_indicator.png'))
        self._placeholder_image = QPixmap(os.path.join(cwd, 'placeholder.png'))

        # Set up ROS interfaces
        self._kill_listener = KillListener()
        self._kill_broadcaster = KillBroadcaster(id = 'PropaGator GUI', 
            description = 'PropaGator GUI kill')
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_cb)
        self._float_sub = rospy.Subscriber('/controller/float_status', Bool, self._float_cb)
        self._float_proxy = rospy.ServiceProxy('/controller/float_mode', FloatMode)
        self._current_controller_sub = rospy.Subscriber('/controller/current_controller', String, self._current_controller_cb)
        self._get_all_controllers_proxy = rospy.ServiceProxy('/controller/get_all_controllers', get_all_controllers)
        self._request_controller_proxy = rospy.ServiceProxy('/controller/request_controller', request_controller)

        # Connect push buttons
        self._kill_push_btn.toggled.connect(self._on_kill_push_btn_toggle)
        self._float_push_btn.toggled.connect(self._on_float_push_btn_toggle)

        # Connect combo boxes
        self._controller_combo_box.activated[str].connect(self._controller_combo_box_cb)

        # Set up update timer at 10Hz
        # A Qt timer is used instead of a ros timer since Qt components are updated
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._onUpdate)
        self.update_timer.start(100)

        # Temp
        self._placeholder_label.setPixmap(self._placeholder_image)

    # Everything needs to be turned off here
    def shutdown_plugin(self):
        self.update_timer.stop()
        self._odom_sub.unregister()
        del self._odom_sub
        self._float_sub.unregister()
        del self._float_sub
        # Kill broadcaster is not cleared, the user should unkill before closing the GUI
        del self._kill_broadcaster
        del self._kill_listener

    # Subscriber callbacks
    # Since this is in a different thread it is possible and likely that
        #   the drawing thread will try and draw while the text is being changed
        #   this causes all kinds of mahem such as segmentation faults, double free, ...
        #   To prevent this from hapening this thread updates only none QT variables
        #   described here http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
    def _odom_cb(self, msg):
        self.last_odom_msg = msg

    def _float_cb(self, status):
        self._float_status = status.data

    def _current_controller_cb(self, controller):
        self._controller = controller.data
        self._current_controller = 'Controller: ' + controller.data

    def _odom_update(self):
        pos = self.last_odom_msg.pose.pose.position
        yaw = quat_to_rotvec(xyzw_array(self.last_odom_msg.pose.pose.orientation))[2]
        vel = self.last_odom_msg.twist.twist.linear
        dYaw = self.last_odom_msg.twist.twist.angular.z

        self._odom_x_label.setText('X: %3.3f' % pos.x)
        self._odom_y_label.setText('Y: %3.3f' % pos.y)
        self._odom_yaw_label.setText('Yaw: %3.3f' % yaw)
        self._odom_d_x_label.setText('dX: %3.3f' % vel.x)
        self._odom_d_y_label.setText('dY: %3.3f' % vel.y)
        self._odom_d_yaw_label.setText('dYaw: %3.3f' % dYaw)

    # Push btn callbacks
    def _on_kill_push_btn_toggle(self, checked):
        if checked:
            self._kill_broadcaster.send(True)
        else:
            self._kill_broadcaster.send(False)

    def _on_float_push_btn_toggle(self, checked):
        # TODO: Check if float service active
        try:
            if checked:
                self._float_proxy(True)
            else:
                self._float_proxy(False)
        except rospy.service.ServiceException:
            pass

    # Combo box callbacks
    def _controller_combo_box_cb(self, text):
        self._request_controller_proxy(text)

    # Update functions
    def _updateStatus(self):
        # Check if killed
        if self._kill_listener.get_killed():
            self._kill_label.setPixmap(self._red_indicator)
        else:
            self._kill_label.setPixmap(self._green_indicator)

        # Check float status
        if self._float_status:
            self._float_label.setPixmap(self._red_indicator)
        else:
            self._float_label.setPixmap(self._green_indicator)            

        # Check if in autonomous or RC
        self._controller_label.setText(self._current_controller)

    def _updateControl(self):
        # Wait until we get the first controller
        if self._controller == '':
            return
            
        controllers = self._get_all_controllers_proxy().controllers
        if controllers != self._controllers:
            self._controllers = controllers
            self._controller_combo_box.clear()
            self._controller_combo_box.addItems(controllers)
        
        index = self._controller_combo_box.findText(self._controller)
        if index != -1 and index != self._controller_combo_box.currentIndex():
            self._controller_combo_box.setCurrentIndex(index)




    def _updateLidar(self):
        pass

    def _onUpdate(self):
        self._updateStatus()
        self._updateControl()
        self._updateLidar()
        self._odom_update()