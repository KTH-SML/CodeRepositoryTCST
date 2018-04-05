# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import yaml
import time
import codecs
import roslaunch
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseWithCovarianceStamped, PoseStamped, PolygonStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem, QVBoxLayout, QComboBox, QLineEdit, QTextBrowser, QPushButton
from python_qt_binding.QtCore import QTimer, Slot, pyqtSlot, QSignalMapper, QRectF, QPointF
from python_qt_binding.QtGui import QImageReader, QImage, QMouseEvent, QCursor, QBrush, QColor, QPixmap, QTransform, QFont

from rqt_simulation.ROS_Publisher import ROS_Publisher
from rqt_simulation.ROS_Subscriber import ROS_Subscriber
from rqt_simulation.CustomComboBox import CustomComboBox

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal

class RobotTab(QWidget):
    def __init__(self, num_robots):
        super(RobotTab, self).__init__()

        # Variables for ROS Publisher
        self.ros_publisher = ROS_Publisher()
        self.agent_type = 'ground'

        self.num_robots = num_robots
        self.robot_name = 'robot' + str(self.num_robots)

        self.layout = QVBoxLayout()
        self.robot_label_name = QLabel(('Robot ' + str(self.num_robots)))
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.robot_label_name.setFont(font)
        self.layout.addWidget(self.robot_label_name)
        self.robot_comboBox = CustomComboBox(self.num_robots-1)
        self.robot_comboBox.addItems(['None', 'TiaGo', 'Turtlebot', 'srd250'])
        self.layout.addWidget(self.robot_comboBox)
        self.robot_comboBox.signalIndexChanged.connect(self.set_agent_type)

        self.robot_label_init = QLabel('Initial pose')
        self.layout.addWidget(self.robot_label_init)
        self.robot_comboBox_init = CustomComboBox(self.num_robots)
        self.layout.addWidget(self.robot_comboBox_init)

        initial_pose_textItem = QGraphicsTextItem('start_' + str(self.num_robots).zfill(2))
        self.initial_pose = {'start_' + str(self.num_robots).zfill(2) : {'label' : 'r01', 'text_item' : initial_pose_textItem}}
        #self.initial_pose_label = 'start_' + str(self.num_robots).zfill(2)
        #self.initial_pose_textItem = QGraphicsTextItem(self.initial_pose_label)

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

        self.robot_label_prefix = QLabel('Planner prefix robot ' + str(self.num_robots))
        self.layout.addWidget(self.robot_label_prefix)
        self.robot_prefix_textbox = QTextBrowser()
        self.layout.addWidget(self.robot_prefix_textbox)
        self.robot_label_sufix = QLabel('Planner sufix robot ' + str(self.num_robots))
        self.layout.addWidget(self.robot_label_sufix)
        self.robot_sufix_textbox = QTextBrowser()
        self.layout.addWidget(self.robot_sufix_textbox)

        self.robot_label_current_goal = QLabel('Current goal robot ' + str(self.num_robots))
        self.layout.addWidget(self.robot_label_current_goal)
        self.robot_current_goal_textbox = QTextBrowser()
        self.layout.addWidget(self.robot_current_goal_textbox)

        self.robot_resend_goal_button = QPushButton('Clear costmap')
        self.layout.addWidget(self.robot_resend_goal_button)


        self.setLayout(self.layout)

        self.init_pose_msg = Pose()
        self.soft_task_msg = String()
        self.hard_task_msg = String()
        self.ros_publisher.add_publisher('/' + self.robot_name + '/init_pose', Pose, 1.0, self.init_pose_msg)
        self.ros_publisher.add_publisher('/' + self.robot_name + '/soft_task', String, 1.0, self.soft_task_msg)
        self.ros_publisher.add_publisher('/' + self.robot_name + '/hard_task', String, 1.0, self.hard_task_msg)

        self.prefix_string = ''
        self.sufix_string = ''

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

        self.last_current_pose = PoseStamped()
        self.last_footprint_point = PointStamped()

        self.ros_publisher.add_publisher('/' + self.robot_name + '/label_marker', Marker, 5.0, self.label_marker_msg)

        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.pose.pose = self.init_pose_msg
        self.current_pose_amcl_subscriber = rospy.Subscriber('/' + self.robot_name + '/amcl_pose', PoseWithCovarianceStamped, self.current_pose_amcl_callback)
        self.current_pose_gazebo_ground_truth_subscriber = rospy.Subscriber('/' + self.robot_name + '/ground_truth/pose_with_covariance', PoseWithCovarianceStamped, self.current_pose_gazebo_ground_truth_callback)
        self.ros_publisher.add_publisher('/' + self.robot_name + '/pose', PoseWithCovarianceStamped, 15.0, self.pose_msg)
        self.local_footprint_subscriber = rospy.Subscriber('/' + self.robot_name + '/move_base/local_costmap/footprint', PolygonStamped, self.local_footprint_callback)

        self.simulation_started = False

        self.clear_costmap = rospy.ServiceProxy('/' + self.robot_name + '/move_base/clear_costmaps', Empty)
        self.robot_resend_goal_button.clicked.connect(self.call_clear_costmap_srvs)

        self.move_base_ac = actionlib.SimpleActionClient('/' + self.robot_name + '/move_base', MoveBaseAction)
        self.robot_current_goal = MoveBaseActionGoal()

    def current_pose_amcl_callback(self, msg):
        self.pose_msg.pose.pose = deepcopy(msg.pose.pose)

        if self.agent_type == 'ground':
            self.label_marker_msg.header = msg.header
            self.label_marker_msg.pose = msg.pose.pose
            self.label_marker_msg.pose.position.z = deepcopy(msg.pose.pose.position.z) + 1
        #print(self.pose_msg)

    def current_pose_gazebo_ground_truth_callback(self, msg):
        if self.agent_type == 'arial':
            self.label_marker_msg.header = msg.header
            self.label_marker_msg.pose = msg.pose.pose
            self.label_marker_msg.pose.position.z = msg.pose.pose.position.z + 1.0

    def local_footprint_callback(self, msg):
        if self.simulation_started:
            if self.agent_type == 'ground':
                msg_point_rounded = Point()
                msg_point_rounded.x = round(msg.polygon.points[0].x, 2)
                msg_point_rounded.y = round(msg.polygon.points[0].y, 2)
                msg_point_rounded.z = round(msg.polygon.points[0].z, 2)

                if msg_point_rounded != self.last_footprint_point.point:
                    self.last_footprint_point.header = msg.header

                if (msg.header.stamp - self.last_footprint_point.header.stamp).to_sec() > 5.0:
                    print('clear')
                    self.clear_costmap()
                    usleep = lambda x: time.sleep(x)
                    usleep(1)
                    self.move_base_ac.send_goal(self.robot_current_goal.goal)
                    self.last_footprint_point.header = msg.header
                self.last_footprint_point.point = msg_point_rounded
            #print(self.last_footprint_point)

    @Slot(bool)
    def call_clear_costmap_srvs(self):
        if self.agent_type == 'ground':
            self.clear_costmap()
            usleep = lambda x: time.sleep(x)
            usleep(1)
            self.move_base_ac.send_goal(self.robot_current_goal.goal)
            print('Costmap cleared')

    def set_agent_type(self):
        if self.robot_comboBox.currentText() == 'srd250':
            self.agent_type = 'arial'
        else:
            self.agent_type = 'ground'

