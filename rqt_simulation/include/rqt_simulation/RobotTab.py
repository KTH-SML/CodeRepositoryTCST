# -*- coding: utf-8 -*-
'''
This is the robot tab. It loads all relevant interactive button and
robot specific ros publisher and ros subscriber
'''

import os
import sys
import rospy
import rospkg
import yaml
import time
import codecs
import roslaunch
import numpy as np
from math import sqrt
from copy import deepcopy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseWithCovarianceStamped, PoseStamped, PolygonStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem, QVBoxLayout, QComboBox, QLineEdit, QTextBrowser, QPushButton, QCheckBox
from python_qt_binding.QtCore import QTimer, Slot, pyqtSlot, QSignalMapper, QRectF, QPointF, pyqtSignal
from python_qt_binding.QtGui import QImageReader, QImage, QMouseEvent, QCursor, QBrush, QColor, QPixmap, QTransform, QFont

from rqt_simulation.ROS_Publisher import ROS_Publisher
from rqt_simulation.ROS_Subscriber import ROS_Subscriber
from rqt_simulation.CustomComboBox import CustomComboBox

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, euler_from_matrix

class RobotTab(QWidget):
    signalRobotNameChanged = pyqtSignal(int)
    def __init__(self, num_robots, robots):
        super(RobotTab, self).__init__()


        # Available robots from gui_config.yaml
        self.robots = robots
        # Set robot type by default to ground
        self.agent_type = 'ground'

        # Number of robots
        self.num_robots = num_robots

        # Set the robot name by default
        self.robot_name = 'robot' + str(self.num_robots)

        # Set layout for tab
        self.layout = QVBoxLayout()

        # Robot tab title
        self.robot_label_name = QLabel(('Robot ' + str(self.num_robots)))
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.robot_label_name.setFont(font)
        self.layout.addWidget(self.robot_label_name)

        # Robot name label and input line
        self.robot_label = QLabel('Robot name')
        self.layout.addWidget(self.robot_label)
        self.robot_name_input = QLineEdit('robot' + str(self.num_robots))
        self.robot_name_input.editingFinished.connect(self.robot_name_changed)
        self.layout.addWidget(self.robot_name_input)

        # Robot model label and comboBox
        self.robot_model_label = QLabel('Robot model')
        self.layout.addWidget(self.robot_model_label)
        self.robot_comboBox = CustomComboBox(self.num_robots-1)
        self.robot_comboBox.addItems(self.robots['Models'].keys())
        self.layout.addWidget(self.robot_comboBox)
        self.robot_comboBox.signalIndexChanged.connect(self.set_agent_type)

        # Initial pose
        self.robot_label_init = QLabel('Initial pose')
        self.layout.addWidget(self.robot_label_init)
        self.robot_comboBox_init = CustomComboBox(self.num_robots)
        self.layout.addWidget(self.robot_comboBox_init)

        # Add initial pose text item to graphic
        initial_pose_textItem = QGraphicsTextItem('start_' + str(self.num_robots).zfill(2))
        self.initial_pose = {'start_' + str(self.num_robots).zfill(2) : {'label' : 'r01', 'text_item' : initial_pose_textItem}}

        # Use Qualisys
        self.robot_localization_label = QLabel('Localization')
        self.layout.addWidget(self.robot_localization_label)
        self.robot_localization_checkBox = QCheckBox('Use Qualisys')
        self.layout.addWidget(self.robot_localization_checkBox)

        # Task specifications
        self.robot_label_task_title = QLabel('Task robot ' + str(self.num_robots))
        self.robot_label_task_title.setFont(font)
        self.layout.addWidget(self.robot_label_task_title)
        self.robot_label_hard_task = QLabel('Hard tasks')
        self.layout.addWidget(self.robot_label_hard_task)
        self.robot_hard_task_input = QLineEdit('([]<> r01) && ([]<> r02)')
        self.layout.addWidget(self.robot_hard_task_input)
        self.robot_label_soft_task = QLabel('Soft tasks')
        self.layout.addWidget(self.robot_label_soft_task)
        self.robot_soft_task_input = QLineEdit()
        self.layout.addWidget(self.robot_soft_task_input)

        # Plan display
        self.robot_label_prefix = QLabel('Planner prefix robot ' + str(self.num_robots))
        self.layout.addWidget(self.robot_label_prefix)
        self.robot_prefix_textbox = QTextBrowser()
        self.layout.addWidget(self.robot_prefix_textbox)
        self.robot_label_sufix = QLabel('Planner sufix robot ' + str(self.num_robots))
        self.layout.addWidget(self.robot_label_sufix)
        self.robot_sufix_textbox = QTextBrowser()
        self.layout.addWidget(self.robot_sufix_textbox)

        # Current goal display
        self.robot_label_current_goal = QLabel('Current goal robot ' + str(self.num_robots))
        self.layout.addWidget(self.robot_label_current_goal)
        self.robot_current_goal_textbox = QTextBrowser()
        self.layout.addWidget(self.robot_current_goal_textbox)

        # Clear costmap button
        self.robot_resend_goal_button = QPushButton('Clear costmap')
        self.layout.addWidget(self.robot_resend_goal_button)

        self.setLayout(self.layout)

        # Messages for publishing pose, soft-task, hard-task and clear_costmap
        self.init_pose_msg = Pose()
        self.soft_task_msg = String()
        self.hard_task_msg = String()

        self.prefix_string = ''
        self.sufix_string = ''

        # Marker displaying robots name
        self.label_marker_msg = Marker()
        self.label_marker_msg.pose = self.init_pose_msg
        self.label_marker_msg.pose.position.z = 1.0
        self.label_marker_msg.text = self.robot_name
        self.label_marker_msg.type = self.label_marker_msg.TEXT_VIEW_FACING
        self.label_marker_msg.id = self.num_robots
        self.label_marker_msg.action = self.label_marker_msg.ADD
        self.label_marker_msg.scale.z = 0.5
        self.label_marker_msg.color.a = 1.0
        self.label_marker_msg.color.r = 0.0
        self.label_marker_msg.color.g = 0.0
        self.label_marker_msg.color.b = 0.0
        self.label_marker_msg.header.frame_id = '/map'

        # Pose msg published for planner
        self.pose_msg = PoseWithCovarianceStamped()

        # Init pose msg for planner
        self.pose_msg.pose.pose = self.init_pose_msg

        # Clear costmap and resend current goal
        self.robot_resend_goal_button.clicked.connect(self.call_clear_costmap_srvs)
        self.robot_current_goal = MoveBaseActionGoal()

        # Start all publisher and Subscriber
        self.start_publisher_and_subscriber()

    # AMCL pose callback
    def current_pose_amcl_callback(self, msg):
        if self.robot_localization_checkBox.checkState() != 2:
            self.pose_msg.pose.pose = deepcopy(msg.pose.pose)
            self.pose_msg.header = deepcopy(msg.header)
            self.pose_msg.header.stamp = rospy.Time.now()

            if self.agent_type == 'ground':
                self.label_marker_msg.header = msg.header
                self.label_marker_msg.pose = msg.pose.pose
                self.label_marker_msg.pose.position.z = deepcopy(msg.pose.pose.position.z) + 1

    # Localization with Qualisys
    def current_pose_qualisys_callback(self, msg):
        if self.robot_localization_checkBox.checkState() == 2:
            self.pose_msg.header = deepcopy(msg.header)
            self.pose_msg.pose.pose = deepcopy(msg.pose)
            self.pose_msg.header.stamp = rospy.Time.now()

            if self.agent_type == 'ground':
                self.label_marker_msg.header = msg.header
                self.label_marker_msg.pose = msg.pose
                self.label_marker_msg.pose.position.z = deepcopy(msg.pose.position.z) + 1

    # Localization with Gazebo ground truth
    def current_pose_gazebo_ground_truth_callback(self, msg):
        if self.agent_type == 'arial':
            self.label_marker_msg.header = msg.header
            self.label_marker_msg.pose = msg.pose.pose
            self.label_marker_msg.pose.position.z = deepcopy(msg.pose.pose.position.z) + 1.0

    # Sends topic to planner to clear costmap manual
    @Slot(bool)
    def call_clear_costmap_srvs(self):
        if self.agent_type == 'ground':
            bool = Bool()
            bool.data = True
            self.clear_costmap_publisher.publish(bool)

    # If robot name in tab has changed the subscriber and publisher topics are updated
    def robot_name_changed(self):
        self.remove_publisher_and_subscriber()
        self.robot_name = self.robot_name_input.text()
        self.start_publisher_and_subscriber()
        self.label_marker_msg.text = self.robot_name
        self.signalRobotNameChanged.emit(self.num_robots)

    # Start all publisher and subscriber from robot tab
    def start_publisher_and_subscriber(self):
        self.ros_publisher = ROS_Publisher()
        self.ros_publisher.add_publisher('/' + self.robot_name + '/init_pose', Pose, 1.0, self.init_pose_msg)
        self.ros_publisher.add_publisher('/' + self.robot_name + '/soft_task', String, 1.0, self.soft_task_msg)
        self.ros_publisher.add_publisher('/' + self.robot_name + '/hard_task', String, 1.0, self.hard_task_msg)
        self.ros_publisher.add_publisher('/' + self.robot_name + '/label_marker', Marker, 5.0, self.label_marker_msg)
        self.clear_costmap_publisher = rospy.Publisher('/' + self.robot_name + '/clear_costmap', Bool, queue_size=1)
        self.current_pose_amcl_subscriber = rospy.Subscriber('/' + self.robot_name + '/amcl_pose', PoseWithCovarianceStamped, self.current_pose_amcl_callback)
        self.current_pose_qualisys_subscriber = rospy.Subscriber('/' + self.robot_name + '/qualisys_pose_map', PoseStamped, self.current_pose_qualisys_callback)
        self.current_pose_gazebo_ground_truth_subscriber = rospy.Subscriber('/' + self.robot_name + '/ground_truth/pose_with_covariance', PoseWithCovarianceStamped, self.current_pose_gazebo_ground_truth_callback)
        self.ros_publisher.add_publisher('/' + self.robot_name + '/pose_gui', PoseWithCovarianceStamped, 15.0, self.pose_msg)

        self.label_marker_msg.text = self.robot_name

    # Remove all publisher and subcriber from robot tab
    def remove_publisher_and_subscriber(self):
        del self.ros_publisher
        del self.clear_costmap_publisher
        del self.current_pose_amcl_subscriber
        del self.current_pose_qualisys_subscriber
        del self.current_pose_gazebo_ground_truth_subscriber

    # Updates robot type if model checkbox was changed
    def set_agent_type(self):
        if self.robot_comboBox.currentText() in self.robots['robot_types']['arial']:
            self.agent_type = 'arial'
        else:
            self.agent_type = 'ground'

