# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import yaml
import codecs
import roslaunch
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray, PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem, QVBoxLayout, QComboBox, QLineEdit, QTextBrowser
from python_qt_binding.QtCore import QTimer, Slot, pyqtSlot, QSignalMapper, QRectF, QPointF
from python_qt_binding.QtGui import QImageReader, QImage, QMouseEvent, QCursor, QBrush, QColor, QPixmap, QTransform, QFont

from rqt_simulation.ROS_Publisher import ROS_Publisher
from rqt_simulation.ROS_Subscriber import ROS_Subscriber
from rqt_simulation.CustomComboBox import CustomComboBox

class RobotTab(QWidget):
    def __init__(self, num_robots):
        super(RobotTab, self).__init__()

        # Variables for ROS Publisher
        self.ros_publisher = ROS_Publisher()

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
        self.robot_comboBox.addItems(['None', 'TiaGo', 'Turtlebot'])
        self.layout.addWidget(self.robot_comboBox)

        self.robot_label_init = QLabel('Initial pose')
        self.layout.addWidget(self.robot_label_init)
        self.robot_comboBox_init = CustomComboBox(self.num_robots)
        self.layout.addWidget(self.robot_comboBox_init)

        self.initial_pose_label = 'start_' + str(self.num_robots).zfill(2)
        self.initial_pose_textItem = QGraphicsTextItem(self.initial_pose_label)

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

        self.ros_publisher.add_publisher('/' + self.robot_name + '/label_marker', Marker, 5.0, self.label_marker_msg)

        self.current_pose_subscriber = rospy.Subscriber('/' + self.robot_name + '/amcl_pose', PoseWithCovarianceStamped, self.current_pose_callback)
        #self.local_plan_sub

        #self.move_base_ac = actionlib.SimpleActionClient('/' + self.robot_name + '/move_base', MoveBaseAction)

    def current_pose_callback(self, msg):
        self.label_marker_msg.header = msg.header
        self.label_marker_msg.pose = msg.pose.pose
        self.label_marker_msg.pose.position.z = 1.0
        print('got_pose')
        if  msg.pose.pose != self.last_current_pose.pose:
            print('now')
            self.last_current_pose.header = msg.header
        print((msg.header.stamp - self.last_current_pose.header.stamp).to_sec())
        if (msg.header.stamp - self.last_current_pose.header.stamp).to_sec() > 5.0:
            print('now2')
        self.last_current_pose.pose = msg.pose.pose

