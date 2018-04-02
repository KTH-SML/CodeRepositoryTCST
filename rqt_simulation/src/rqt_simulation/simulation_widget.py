#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from subprocess import call
import sys
import rospy
import rospkg
import yaml
import codecs
import roslaunch
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
from math import pi

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem, QVBoxLayout, QComboBox, QLineEdit, QTextBrowser
from python_qt_binding.QtCore import QTimer, Slot, pyqtSlot, QSignalMapper, QRectF, QPointF
from python_qt_binding.QtGui import QImageReader, QImage, QMouseEvent, QCursor, QBrush, QColor, QPixmap, QTransform, QFont

from rqt_simulation.map_dialog import Map_dialog
from .initial_pose import Initial_pose
from rqt_simulation.MapGraphicsScene import MapGraphicsScene
from rqt_simulation.ROS_Subscriber import ROS_Subscriber
from rqt_simulation.CustomComboBox import CustomComboBox
from rqt_simulation.RVIZFileGenerator import RVIZFileGenerator
from rqt_simulation.ROS_Publisher import ROS_Publisher
from rqt_simulation.RobotTab import RobotTab

from ltl_tools.planner import ltl_planner
from ltl_tools.ts import MotionFts, ActionModel, MotActModel

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal

from pyquaternion import Quaternion


class SimulationWidget(QWidget):

    def __init__(self):
        super(SimulationWidget, self).__init__()
        self.setObjectName('SimulationWidget')

        # Load ui file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'SimulationPlugin.ui')
        loadUi(ui_file, self)

        # Connect buttons from ui file with functions
        self.button_RI.pressed.connect(self.on_button_RI_pressed)                       # Select ROI and FTS button
        self.button_setup.clicked.connect(self.on_button_setup_pressed)                 # Setup Simulation button
        self.button_setup_exp.clicked.connect(self.on_button_setup_exp_pressed)         # Setup Experiment button
        self.button_execute_task.clicked.connect(self.on_button_execute_task_pressed)   # Synthesize task button
        self.button_addRobot.clicked.connect(self.add_robot)                            # Add robot tab button
        self.button_remove_robot.clicked.connect(self.remove_robot)                     # Remove robot button
        self.button_record_rosbag.clicked.connect(self.on_button_rosbag_clicked)
        self.button_start_sim.clicked.connect(self.on_button_start_sim_pressed)         # Start simulation button
        self.world_comboBox.currentIndexChanged.connect(self.reset)                     # World combobox

        # Disable buttons
        self.button_setup.setEnabled(False)
        self.button_setup_exp.setEnabled(False)
        self.button_remove_robot.setEnabled(False)
        self.button_start_sim.setEnabled(False)
        self.button_execute_task.setEnabled(False)

        # Variables for ROS Publisher
        self.ros_publisher = ROS_Publisher()

        # Robot tab variables
        self.num_robots = 0
        self.tab_list = []

        self.rosbag_active = False

        # Subscriber for prefix and sufix
        self.prefix_plan_topic_list = []
        self.prefix_plan_subscriber_list = []
        self.sufix_plan_topic_list = []
        self.sufix_plan_subscriber_list = []
        self.current_goal_topic_list = []
        self.current_goal_subscriber_list = []

        self.prefix_string = ''
        self.sufix_string = ''
        self.current_goal_string = ''

        # Initialize GraphicsScene
        self.current_graphicsScene = MapGraphicsScene()
        self.graphicsView_main.setScene(self.current_graphicsScene)

        # Items for displaying ROIs
        #self.ellipse_items_RI = []
        #self.ellipse_items_labels_RI = []
        self.initial_pose_textItem_list = []
        self.initial_pose = {}
        self.region_of_interest = {}
        self.green_ellipse_list = []

        # Items for displaying FTS
        #self.line_dict = {}
        #self.arrow_list = []

        # Load map image
        self.scenario = self.world_comboBox.currentText()
        self.current_graphicsScene.load_map(self.scenario)
        #map_yaml = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, 'map.yaml')
        #self.loadConfig(map_yaml)
        #if self.scenario == 'pal_office' or self.scenario == 'sml':
        #    map = 'map.pgm'
        #else:
        #    map = 'map.png'

        #map_file = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, map)
        #pixmap = QPixmap(map_file)
        #mapSize = pixmap.size()
        #self.current_graphicsScene.addPixmap(pixmap)

        # Add world origin
        #self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + mapSize.height())
        #self.current_graphicsScene.addCoordinateSystem(self.worldOrigin, 0.0)

        # Scale map
        rectF = self.graphicsView_main.geometry()
        if (float(rectF.width())/self.current_graphicsScene.mapSize.width() < float(rectF.height())/self.current_graphicsScene.mapSize.height()):
            scale = float(rectF.width())/self.current_graphicsScene.mapSize.width()
        elif self.scenario == 'pal_office' or self.scenario == 'sml':
            scale = 0.7
        else:
            scale = float(rectF.height())/self.current_graphicsScene.mapSize.height()
        transform = QTransform(scale, 0, 0.0, scale, 0, 0)
        self.graphicsView_main.setTransform(transform)

        #self.map_dialog = Map_dialog(self.current_graphicsScene)

        # ROI marker msg
        self.region_pose_marker_array_msg = MarkerArray()

        # Add robot tab
        self.add_robot()

        # Publisher to set ltl_planner active
        self.start_publisher = rospy.Publisher('/planner_active', Bool, queue_size = 1)

        # Counter for marker id counter
        self.marker_id_counter = 0

    # Reset is called if new map selected
    def reset(self):

        # Disable buttons
        self.button_setup.setEnabled(False)
        self.button_start_sim.setEnabled(False)
        self.button_execute_task.setEnabled(False)

        # Reinitialize map and clear all item lists
        self.current_graphicsScene = MapGraphicsScene()
        self.graphicsView_main.setScene(self.current_graphicsScene)
        #self.ellipse_items_RI = []
        #self.ellipse_items_labels_RI = []
        self.initial_pose_textItem_list = []
        for i in range(0, self.num_robots):
            self.tab_list[i].robot_comboBox_init.clear()
            self.tab_list[i].initial_pose_textItem = QGraphicsTextItem(self.tab_list[i].initial_pose_label)
            self.tab_list[i].robot_sufix_textbox.clear()
            self.tab_list[i].robot_prefix_textbox.clear()

        self.initial_pose = {}
        self.region_of_interest = {}

        #self.line_dict = {}
        self.prefix_string = ''
        self.sufix_string = ''
        #self.arrow_list = []

        self.scenario = self.world_comboBox.currentText()
        self.current_graphicsScene.load_map(self.scenario)
        #self.map_dialog = Map_dialog(self.current_graphicsScene)
        #map_yaml = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, 'map.yaml')
        #self.loadConfig(map_yaml)
        #if self.scenario == 'pal_office' or self.scenario == 'sml':
        #    map = 'map.pgm'
        #else:
        #    map = 'map.png'

        #map_file = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, map)
        #pixmap = QPixmap(map_file)
        #mapSize = pixmap.size()
        #self.current_graphicsScene.addPixmap(pixmap)

        #self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + mapSize.height())
        #self.current_graphicsScene.addCoordinateSystem(self.worldOrigin, 0.0)

        rectF = self.graphicsView_main.geometry()
        if (float(rectF.width())/self.current_graphicsScene.mapSize.width() < float(rectF.height())/self.current_graphicsScene.mapSize.height()):
           scale = float(rectF.width())/self.current_graphicsScene.mapSize.width()
        else:
           scale = float(rectF.height())/self.current_graphicsScene.mapSize.height()
        transform = QTransform(scale, 0, 0.0, scale, 0, 0)
        self.graphicsView_main.setTransform(transform)

        # Reinitialize ROI marker msg
        self.region_pose_marker_array_msg = MarkerArray()


    # Callback for prefix from ltl_planner
    def prefix_callback(self, msg, source):
        for n in msg.poses:
            for i in range(0, len(self.region_of_interest)):
                if self.position_msg_to_tuple(n.position) == self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position']:
                    self.prefix_string =  self.prefix_string + self.region_of_interest.keys()[i] + ' --> '
        index = self.prefix_plan_topic_list.index(source)
        # Send signal for received msg
        self.prefix_plan_subscriber_list[index].received.emit(index)

    # Fill prefix in textbox
    @pyqtSlot(int)
    def received_prefix(self, index):
        self.tab_list[index].robot_prefix_textbox.clear()
        self.tab_list[index].robot_prefix_textbox.insertPlainText('Prefix: ')
        self.tab_list[index].robot_prefix_textbox.insertPlainText(self.prefix_string)
        self.prefix_string = ''
        self.button_setup.setEnabled(True)
        self.button_setup_exp.setEnabled(True)

    # Callback for sufix from ltl_planner
    def sufix_callback(self, msg, source):
        for n in msg.poses:
            for i in range(0, len(self.region_of_interest)):
                if self.position_msg_to_tuple(n.position) == self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position']:
                    self.sufix_string = self.sufix_string + self.region_of_interest.keys()[i] + ' --> '
        index = self.sufix_plan_topic_list.index(source)
        # Send signal for recieved msg
        self.sufix_plan_subscriber_list[index].received.emit(index)

    # Fill sufix in textbox
    @pyqtSlot(int)
    def received_sufix(self, index):
        self.tab_list[index].robot_sufix_textbox.clear()
        self.tab_list[index].robot_sufix_textbox.insertPlainText('Sufix: ')
        self.tab_list[index].robot_sufix_textbox.insertPlainText(self.sufix_string)
        self.sufix_string = ''

    def goal_callback(self, msg, source):
        for i in range(0, len(self.region_of_interest)):
            if self.position_msg_to_tuple(msg.goal.target_pose.pose.position) == self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position']:
                self.current_goal_string = self.region_of_interest.keys()[i]
        index = self.current_goal_topic_list.index(source)
        # Send signal for recieved msg
        self.current_goal_subscriber_list[index].received.emit(index)
        self.tab_list[index].robot_current_goal = msg

    @pyqtSlot(int)
    def received_goal(self, index):
        self.tab_list[index].robot_current_goal_textbox.clear()
        self.tab_list[index].robot_current_goal_textbox.insertPlainText('Current goal: ')
        self.tab_list[index].robot_current_goal_textbox.insertPlainText(self.current_goal_string)

    @Slot(bool)
    def on_button_RI_pressed(self):
        # Check if map is empty and remove items
        #graphicScene_item = self.current_graphicsScene.items()
        #if len(graphicScene_item) > 9:
        #    for i in range(0, len(self.ellipse_items_RI)):
        #        self.current_graphicsScene.removeItem(self.ellipse_items_RI[i])
        #        self.current_graphicsScene.removeItem(self.ellipse_items_labels_RI[i])
        #    for i in range(0, self.num_robots):
        #        self.current_graphicsScene.removeItem(self.tab_list[i].initial_pose_textItem)
        #    for i in range(0, len(self.line_dict)):
        #        self.current_graphicsScene.removeItem(self.line_dict[self.line_dict.keys()[i]])
        #    for i in range(0, len(self.arrow_list)):
        #        self.current_graphicsScene.removeArrow(self.arrow_list[i])
        #    for i in range(0, self.num_robots):
        #        self.tab_list[i].robot_sufix_textbox.clear()
        #        self.tab_list[i].robot_prefix_textbox.clear()
        #        self.tab_list[i].robot_comboBox_init.clear()

        # Start map dialog
        map_dialog = Map_dialog(self.current_graphicsScene)
        map_dialog.exec_()

        # Copy selected ROIs and FTS
        #self.ellipse_items_RI = map_dialog.ellipse_items
        #self.ellipse_items_labels_RI = map_dialog.ellipse_items_labels
        self.region_of_interest = map_dialog.region_of_interest
        #self.pixel_coords = map_dialog.pixel_coords_list
        self.region_list = map_dialog.region_list
        #self.add_region_marker(self.region_of_interest, False)
        #self.line_dict = map_dialog.line_dict
        #self.arrow_list = map_dialog.arrow_list

        # Add initial poses
        if len(self.current_graphicsScene.ellipse_items) > 0:
            for i in range(0, self.num_robots):
                self.tab_list[i].robot_comboBox_init.clear()
                #self.current_graphicsScene.addItem(self.tab_list[i].initial_pose_textItem)
                for j in range(0, len(self.region_of_interest)):
                    self.tab_list[i].robot_comboBox_init.addItem(self.region_of_interest.keys()[j])
                self.tab_list[i].robot_comboBox_init.model().sort(0)

            self.button_execute_task.setEnabled(True)

    @pyqtSlot(int, int)
    def set_init_pose_id(self, index, id):
        if self.tab_list[id -1].robot_comboBox_init.count() > 0:
            self.initial_pose['start_' + str(id)] = self.region_of_interest[self.tab_list[id -1].robot_comboBox_init.currentText()]
            self.tab_list[id -1].init_pose_msg.position.x = self.initial_pose['start_' + str(id)]['pose']['position'][0]
            self.tab_list[id -1].init_pose_msg.position.y = self.initial_pose['start_' + str(id)]['pose']['position'][1]
            self.tab_list[id -1].init_pose_msg.position.z = self.initial_pose['start_' + str(id)]['pose']['position'][2]
            self.tab_list[id -1].init_pose_msg.orientation.w = self.initial_pose['start_' + str(id)]['pose']['orientation'][0]
            self.tab_list[id -1].init_pose_msg.orientation.x = self.initial_pose['start_' + str(id)]['pose']['orientation'][1]
            self.tab_list[id -1].init_pose_msg.orientation.y = self.initial_pose['start_' + str(id)]['pose']['orientation'][2]
            self.tab_list[id -1].init_pose_msg.orientation.z = self.initial_pose['start_' + str(id)]['pose']['orientation'][3]

            index = self.region_list.index(self.tab_list[id -1].robot_comboBox_init.currentText())
            self.green_ellipse_list[id-1] = index
            for i in range(0, len(self.region_list)):
                if i in self.green_ellipse_list:
                    self.current_graphicsScene.ellipse_items[i].setBrush(QBrush(QColor('green')))
                else:
                    self.current_graphicsScene.ellipse_items[i].setBrush(QBrush(QColor('red')))
                if i == index:
                    rect = self.current_graphicsScene.ellipse_items[i].rect()
                    point = rect.topLeft()
                    self.tab_list[id -1].initial_pose_textItem.setPos(point.x() - 11, point.y() - 22)


    @Slot(bool)
    def on_button_setup_pressed(self):
        scenario = self.world_comboBox.currentText()

        # Disable buttons
        self.button_RI.setEnabled(False)
        self.button_setup.setEnabled(False)
        self.button_remove_robot.setEnabled(False)
        self.world_comboBox.setEnabled(False)
        self.button_start_sim.setEnabled(True)
        self.button_addRobot.setEnabled(False)

        # Get robot types to generate RVIZ file
        robot_list = []
        for i in range(0, self.num_robots):
            if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
                robot_list.append('tiago')
            elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
                robot_list.append('turtlebot')
            elif self.tab_list[i].robot_comboBox.currentText() == 'srd250':
                robot_list.append('srd250')
        file = RVIZFileGenerator(robot_list)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Launch gazebo world
        launch_world = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'setup_simulation.launch')])
        sys.argv.append('scenario:=' + scenario)
        launch_world.start()
        del sys.argv[2:len(sys.argv)]

        # Launch rosbag logger
        if self.rosbag_active == True:
            launch_logger = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rosbag_writer.launch')])
            sys.argv.append('num_robots:=' + str(self.num_robots))
            launch_logger.start()
            del sys.argv[2:len(sys.argv)]

        # Launch robots
        launch_robot_list = []
        for i in range(0, self.num_robots):
            launch_robot_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot.launch')]))
            quaternion = Quaternion(self.initial_pose['start_' + str(i+1)]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]
            print('theta launch')
            print(rot_axis)
            print(theta)
            print(quaternion)
            if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
                sys.argv.append('robot_model:=tiago_steel')
            elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
                sys.argv.append('robot_model:=turtlebot')
            elif self.tab_list[i].robot_comboBox.currentText() == 'srd250':
                sys.argv.append('robot_model:=srd250')
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('initial_pose_x:=' + str(self.initial_pose['start_' + str(i+1)]['pose']['position'][0]))
            sys.argv.append('initial_pose_y:=' + str(self.initial_pose['start_' + str(i+1)]['pose']['position'][1]))
            sys.argv.append('initial_pose_a:=' + str(theta))
            sys.argv.append('scenario:=' + scenario)

            launch_robot_list[i].start()
            navigation = actionlib.SimpleActionClient('/' + self.tab_list[i].robot_name + '/move_base', MoveBaseAction)
            rospy.loginfo("wait for the move_base action server to come up")
            #allow up to 5 seconds for the action server to come up
            #navigation.wait_for_server(rospy.Duration(5))
            #wait for the action server to come up
            #navigation.wait_for_server()
            del sys.argv[2:len(sys.argv)]

            rospy.loginfo("server up")

        # Publish region marker
        self.add_region_marker(self.region_of_interest, False)
        self.add_region_marker(self.initial_pose, True)
        self.ros_publisher.add_publisher('region_of_interest', MarkerArray, 1.0, self.region_pose_marker_array_msg)

    @Slot(bool)
    def on_button_setup_exp_pressed(self):
        scenario = self.world_comboBox.currentText()

        # Disable buttons
        self.button_RI.setEnabled(False)
        self.button_setup.setEnabled(False)
        self.button_remove_robot.setEnabled(False)
        self.world_comboBox.setEnabled(False)
        self.button_start_sim.setEnabled(True)
        self.button_addRobot.setEnabled(False)

        #Get robot types to generate RVIZ file
        robot_list = []
        for i in range(0, self.num_robots):
            if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
                robot_list.append('tiago')
            elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
                robot_list.append('turtlebot')
        file = RVIZFileGenerator(robot_list)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Launch rosbag logger
        if self.rosbag_active == True:
            launch_logger = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rosbag_writer.launch')])
            sys.argv.append('num_robots:=' + str(self.num_robots))
            launch_logger.start()
            del sys.argv[2:len(sys.argv)]

        # Launch rviz
        launch_world = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rviz.launch')])
        launch_world.start()

        # Launch robots
        launch_robot_list = []
        for i in range(0, self.num_robots):
            launch_robot_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot_exp.launch')]))
            quaternion = Quaternion(self.initial_pose['start_' + str(i+1)]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]
            #if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
            #    sys.argv.append('robot_model:=tiago_steel')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
            #    sys.argv.append('robot_model:=turtlebot')
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('initial_pose_x:=' + str(self.initial_pose['start_' + str(i+1)]['pose']['position'][0]))
            sys.argv.append('initial_pose_y:=' + str(self.initial_pose['start_' + str(i+1)]['pose']['position'][1]))
            sys.argv.append('initial_pose_a:=' + str(theta))
            sys.argv.append('scenario:=' + scenario)

            launch_robot_list[i].start()
            #navigation = actionlib.SimpleActionClient('/' + self.tab_list[i].robot_name + '/move_base', MoveBaseAction)
            #rospy.loginfo("wait for the move_base action server to come up")
            #allow up to 5 seconds for the action server to come up
            #navigation.wait_for_server(rospy.Duration(5))
            #wait for the action server to come up
            #navigation.wait_for_server()
            del sys.argv[2:len(sys.argv)]

            rospy.loginfo("server up")

        # Publish region marker
        self.add_region_marker(self.region_of_interest, False)
        self.add_region_marker(self.initial_pose, True)
        self.ros_publisher.add_publisher('region_of_interest', MarkerArray, 1.0, self.region_pose_marker_array_msg)

    @Slot(bool)
    def on_button_execute_task_pressed(self):
        print('saved task')
        #task_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'task', 'task.yaml')
        #tasks1 = {'hard_task' : self.hard_task_input.text(), 'soft_task' : self.soft_task_input.text()}
        #robot_task = {}
        #robot_task['tiago1'] = tasks1
        #tasks2 = {'hard_task' : self.robot2_hard_task_input.text(), 'soft_task' : self.robot2_soft_task_input.text()}
        #robot_2_task = {}
        #robot_task['tiago2'] = tasks2
        #data = robot_task
        #with codecs.open(task_file, 'w', encoding='utf-8') as outfile:
        #    yaml.safe_dump(data, outfile, default_flow_style=False)

        # Launch ltl_planner
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_task_list = []
        for i in range(0, self.num_robots):
            self.tab_list[i].soft_task_msg.data = self.tab_list[i].robot_soft_task_input.text()
            self.tab_list[i].hard_task_msg.data = self.tab_list[i].robot_hard_task_input.text()
            roslaunch_task_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'ltl_planner.launch')]))
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('agent_type:=' + self.tab_list[i].agent_type)
            roslaunch_task_list[i].start()
            del sys.argv[2:len(sys.argv)]

    @Slot(bool)
    def on_button_start_sim_pressed(self):
        start_msg = Bool()
        start_msg.data = True
        self.start_publisher.publish(start_msg)
        for i in range(0, self.num_robots):
            self.tab_list[i].simulation_started = True

    @Slot(bool)
    def on_button_rosbag_clicked(self):
        if self.rosbag_active == False:
            self.rosbag_active = True
            self.button_record_rosbag.setDown(True)
        elif self.rosbag_active == True:
            self.rosbag_active = False
            self.button_record_rosbag.setDown(False)

    def position_msg_to_tuple(self, position_msg):
        position = (position_msg.x, position_msg.y, position_msg.z)
        return position

    @Slot(bool)
    def add_robot(self):
        self.num_robots += 1
        self.tab_list.append(RobotTab(self.num_robots))
        self.tabWidget.addTab(self.tab_list[self.num_robots-1], ('Robot ' + str(self.num_robots)))
        self.button_remove_robot.setEnabled(True)

        self.current_graphicsScene.addItem(self.tab_list[self.num_robots-1].initial_pose_textItem)

        if self.num_robots > 1:
           for i in range(0, len(self.region_of_interest)):
               self.tab_list[self.num_robots-1].robot_comboBox_init.addItem(self.region_of_interest.keys()[i])
           self.tab_list[self.num_robots-1].robot_comboBox_init.model().sort(0)
        if len(self.current_graphicsScene.ellipse_items) > 0:
           self.tab_list[self.num_robots-1].init_pose_msg.position.x = self.region_of_interest[self.region_of_interest.keys()[0]]['pose']['position'][0]
           self.tab_list[self.num_robots-1].init_pose_msg.position.y = self.region_of_interest[self.region_of_interest.keys()[0]]['pose']['position'][1]
           self.tab_list[self.num_robots-1].init_pose_msg.position.z = self.region_of_interest[self.region_of_interest.keys()[0]]['pose']['position'][2]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.w = self.region_of_interest[self.region_of_interest.keys()[0]]['pose']['orientation'][0]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.x = self.region_of_interest[self.region_of_interest.keys()[0]]['pose']['orientation'][1]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.y = self.region_of_interest[self.region_of_interest.keys()[0]]['pose']['orientation'][2]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.z = self.region_of_interest[self.region_of_interest.keys()[0]]['pose']['orientation'][3]
           self.current_graphicsScene.ellipse_items[0].setBrush(QBrush(QColor('green')))
           rect = self.current_graphicsScene.ellipse_items[0].rect()
           point = rect.topLeft()
           self.tab_list[self.num_robots-1].initial_pose_textItem.setPos(point.x() - 11, point.y() - 22)
        self.tab_list[self.num_robots-1].robot_comboBox_init.signalIndexChanged.connect(self.set_init_pose_id)

        self.prefix_plan_topic_list.append('/' + self.tab_list[self.num_robots-1].robot_name + '/prefix_plan')
        self.prefix_plan_subscriber_list.append(ROS_Subscriber(self.prefix_plan_topic_list[self.num_robots-1], PoseArray, self.prefix_callback))
        self.sufix_plan_topic_list.append('/' + self.tab_list[self.num_robots-1].robot_name + '/sufix_plan')
        self.sufix_plan_subscriber_list.append(ROS_Subscriber(self.sufix_plan_topic_list[self.num_robots-1], PoseArray, self.sufix_callback))

        self.prefix_plan_subscriber_list[self.num_robots-1].received.connect(self.received_prefix)
        self.sufix_plan_subscriber_list[self.num_robots-1].received.connect(self.received_sufix)

        self.current_goal_topic_list.append('/' + self.tab_list[self.num_robots-1].robot_name + '/move_base/goal')
        self.current_goal_subscriber_list.append(ROS_Subscriber(self.current_goal_topic_list[self.num_robots-1], MoveBaseActionGoal, self.goal_callback))
        self.current_goal_subscriber_list[self.num_robots-1].received.connect(self.received_goal)

        self.green_ellipse_list.append(0)



    @Slot(bool)
    def remove_robot(self):
        if self.num_robots > 1:
            self.num_robots = self.num_robots - 1

            del self.current_goal_topic_list[self.num_robots]
            del self.current_goal_subscriber_list[self.num_robots]
            del self.prefix_plan_subscriber_list[self.num_robots]
            del self.sufix_plan_subscriber_list[self.num_robots]
            del self.prefix_plan_topic_list[self.num_robots]
            del self.sufix_plan_topic_list[self.num_robots]

            self.current_graphicsScene.removeItem(self.tab_list[self.num_robots].initial_pose_textItem)

            #if len(self.region_of_interest) > 1:
                #self.ellipse_items_RI[self.green_ellipse_list[self.num_robots]].setBrush(QBrush(QColor('red')))
            del self.green_ellipse_list[self.num_robots]

            if self.region_list > 0:
                for i in range(0, len(self.region_list)):
                    if i in self.green_ellipse_list:
                        self.ellipse_items_RI[i].setBrush(QBrush(QColor('green')))
                    else:
                        self.ellipse_items_RI[i].setBrush(QBrush(QColor('red')))

            self.tabWidget.removeTab(self.num_robots)
            del self.tab_list[self.num_robots]

            if self.num_robots == 1:
                self.button_remove_robot.setEnabled(False)


    def add_region_marker(self, region, initial):

        for i in range(0, len(region)):
            pose_marker = Pose()
            pose_text = Pose()
            self.region_pose_marker = Marker()
            self.region_pose_marker_label = Marker()

            self.region_pose_marker.pose = pose_marker
            self.region_pose_marker_label.pose = pose_text

            pose_marker.position.x = region[region.keys()[i]]['pose']['position'][0]
            pose_marker.position.y = region[region.keys()[i]]['pose']['position'][1]
            pose_text.position.x = region[region.keys()[i]]['pose']['position'][0]
            pose_text.position.y = region[region.keys()[i]]['pose']['position'][1]

            if initial:
                self.region_pose_marker_label.text = region.keys()[i]
                self.region_pose_marker.color.r = 0.0
                self.region_pose_marker.color.g = 0.5
                self.region_pose_marker.color.b = 0.0
                self.region_pose_marker.pose.position.z = 0.01
                self.region_pose_marker_label.pose.position.z = 1.5
                self.region_pose_marker.scale.x = 0.5
                self.region_pose_marker.scale.y = 0.5
            else:
                self.region_pose_marker_label.text = region.keys()[i]
                self.region_pose_marker.color.r = 0.5
                self.region_pose_marker.color.g = 0.0
                self.region_pose_marker.color.b = 0.0
                self.region_pose_marker_label.pose.position.z = 0.5

                self.region_pose_marker.scale.x = 1.0
                self.region_pose_marker.scale.y = 1.0

            self.region_pose_marker.header.frame_id = '/map'

            self.region_pose_marker.type = self.region_pose_marker.CYLINDER
            self.region_pose_marker.id = self.marker_id_counter
            self.region_pose_marker.action = self.region_pose_marker.ADD
            self.region_pose_marker.scale.z = 0.01
            self.region_pose_marker.color.a = 1.0

            self.region_pose_marker_label.header.frame_id = '/map'

            self.region_pose_marker_label.type = self.region_pose_marker.TEXT_VIEW_FACING
            self.region_pose_marker_label.id = self.marker_id_counter + 1
            self.region_pose_marker_label.action = self.region_pose_marker.ADD
            self.region_pose_marker_label.scale.z = 0.5
            self.region_pose_marker_label.color.a = 1.0
            self.region_pose_marker_label.color.r = 0.0
            self.region_pose_marker_label.color.g = 0.0
            self.region_pose_marker_label.color.b = 0.0

            self.region_pose_marker_array_msg.markers.append(self.region_pose_marker_label)
            self.region_pose_marker_array_msg.markers.append(self.region_pose_marker)

            self.marker_id_counter = self.marker_id_counter + 2

    def loadConfig(self, filename):
        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()
        self.map_image = data['image']
        self.map_resolution = data['resolution']
        self.map_origin = tuple(data['origin'])
        self.map_negate = data['negate']
        self.map_occupied_thresh = data['occupied_thresh']
        self.map_free_thresh = data['free_thresh']
        rospy.loginfo('rqt_simulation map : %s' % (self.scenario))
        rospy.loginfo('rqt_simulation map resolution : %.6f' % (self.map_resolution))
        rospy.loginfo('rqt_simulation map origin : %s' % (self.map_origin,))
        rospy.loginfo('rqt_simulation map negate : %s' % (self.map_negate))
        rospy.loginfo('rqt_simulation map occupied threshold : %s' % (self.map_occupied_thresh))
        rospy.loginfo('rqt_simulation map free threshold : %s' % (self.map_free_thresh))
