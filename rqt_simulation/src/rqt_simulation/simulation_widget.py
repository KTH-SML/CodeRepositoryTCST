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
from math import atan2, cos, sin, pi, atan

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem, QVBoxLayout, QComboBox, QLineEdit, QTextBrowser, QGridLayout, QFileDialog
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
        self.button_load_scenario.clicked.connect(self.load_scenario)

        # Disable buttons
        self.button_setup.setEnabled(False)
        self.button_setup_exp.setEnabled(False)
        self.button_remove_robot.setEnabled(False)
        self.button_start_sim.setEnabled(False)
        self.button_execute_task.setEnabled(False)
        self.button_record_rosbag.setEnabled(False)

        # Load configuration available robots and worlds
        config_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'gui_config.yaml')
        stream = file(config_file, 'r')
        data = yaml.load(stream)

        self.robots = data['Robots']
        worlds = data['Worlds']

        # Initialize FTS
        self.FTS = FTS()

        self.scenario_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')

        # Variables for ROS Publisher
        self.ros_publisher = ROS_Publisher()

        # Robot tab variables
        self.num_robots = 0
        self.tab_list = []

        self.world_comboBox.addItems(worlds)

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

        # Load map image
        self.scenario = self.world_comboBox.currentText()
        self.current_graphicsScene.load_map(self.scenario)

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

        # ROI marker msg
        self.region_pose_marker_array_msg = MarkerArray()

        # Add robot tab
        self.add_robot()

        # Publisher to set ltl_planner active
        self.start_publisher = rospy.Publisher('/planner_active', Bool, queue_size = 1)

        # Publisher to set logger active
        self.logger_active_msg = Bool()
        self.logger_active_msg.data = False
        self.ros_publisher.add_publisher('/logger_active', Bool, 1.0, self.logger_active_msg)

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

        self.scenario = self.world_comboBox.currentText()
        self.current_graphicsScene.load_map(self.scenario)

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

        for i in range(0, self.num_robots):
            self.tab_list[i].robot_comboBox_init.clear()
            self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'] = QGraphicsTextItem('start_' + str(i+1).zfill(2))
            self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'].setPos(0.0, 0.0)
            self.current_graphicsScene.addItem(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'])
            self.tab_list[i].robot_sufix_textbox.clear()
            self.tab_list[i].robot_prefix_textbox.clear()

        self.prefix_string = ''
        self.sufix_string = ''

        self.FTS = FTS()

        self.scenario_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')

        # Reinitialize ROI marker msg
        self.region_pose_marker_array_msg = MarkerArray()


    # Callback for prefix from ltl_planner
    def prefix_callback(self, msg, source):
        for n in msg.poses:
            for i in range(0, len(self.FTS.region_of_interest)):
                if self.position_msg_to_tuple(n.position) == tuple(self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[i]]['pose']['position']):
                    self.prefix_string =  self.prefix_string + self.FTS.region_of_interest.keys()[i] + ' --> '
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
            for i in range(0, len(self.FTS.region_of_interest)):
                if self.position_msg_to_tuple(n.position) == tuple(self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[i]]['pose']['position']):
                    self.sufix_string = self.sufix_string + self.FTS.region_of_interest.keys()[i] + ' --> '
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

    # Callback for current goal
    def goal_callback(self, msg, source):
        for i in range(0, len(self.FTS.region_of_interest)):
            if self.position_msg_to_tuple(msg.goal.target_pose.pose.position) == tuple(self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[i]]['pose']['position']):
                self.current_goal_string = self.FTS.region_of_interest.keys()[i]
        index = self.current_goal_topic_list.index(source)
        # Send signal for recieved msg
        self.current_goal_subscriber_list[index].received.emit(index)
        self.tab_list[index].robot_current_goal = msg

    # Fill current goal in textbox
    @pyqtSlot(int)
    def received_goal(self, index):
        self.tab_list[index].robot_current_goal_textbox.clear()
        self.tab_list[index].robot_current_goal_textbox.insertPlainText('Current goal: ')
        self.tab_list[index].robot_current_goal_textbox.insertPlainText(self.current_goal_string)

    @Slot(bool)
    def on_button_RI_pressed(self):
        # Start map dialog
        map_dialog = Map_dialog(self.current_graphicsScene, self.FTS)
        map_dialog.exec_()

        # Add initial poses
        if len(self.current_graphicsScene.items_dict) > 0:
            for i in range(0, self.num_robots):
                self.tab_list[i].robot_comboBox_init.clear()
                for j in range(0, len(self.FTS.region_of_interest)):
                    self.tab_list[i].robot_comboBox_init.addItem(self.FTS.region_of_interest.keys()[j])
                self.tab_list[i].robot_comboBox_init.model().sort(0)

            self.button_execute_task.setEnabled(True)

    # Change initial pose if checkBox entry changed
    # index: Is Checkbox index to set initial pose
    # id: Is the ID of the robot tab, for changing the initial pose of the right robot
    @pyqtSlot(int, int)
    def set_init_pose_id(self, index, id):
        if self.tab_list[id -1].robot_comboBox_init.count() > 0:

            # Get the initial pose
            self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose'] = self.FTS.region_of_interest[self.tab_list[id -1].robot_comboBox_init.currentText()]['pose']
            self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['label'] = self.tab_list[id -1].robot_comboBox_init.currentText()

            # Save it in initial pose message
            self.tab_list[id -1].init_pose_msg.position.x = self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose']['position'][0]
            self.tab_list[id -1].init_pose_msg.position.y = self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose']['position'][1]
            self.tab_list[id -1].init_pose_msg.position.z = self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose']['position'][2]
            self.tab_list[id -1].init_pose_msg.orientation.w = self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose']['orientation'][0]
            self.tab_list[id -1].init_pose_msg.orientation.x = self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose']['orientation'][1]
            self.tab_list[id -1].init_pose_msg.orientation.y = self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose']['orientation'][2]
            self.tab_list[id -1].init_pose_msg.orientation.z = self.tab_list[id -1].initial_pose['start_' + str(id).zfill(2)]['pose']['orientation'][3]

            # Update graphics scene
            for i in range(0, len(self.current_graphicsScene.items_dict)):
                self.current_graphicsScene.items_dict[self.current_graphicsScene.items_dict.keys()[i]]['ellipse_item'].setBrush(QBrush(QColor('red')))

            for i in range(0, self.num_robots):
                self.current_graphicsScene.items_dict[self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['label']]['ellipse_item'].setBrush(QBrush(QColor('green')))
                rect = self.current_graphicsScene.items_dict[self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['label']]['ellipse_item'].rect()
                point = rect.topLeft()
                self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'].setPos(point.x() - 11, point.y() - 22)

    @Slot(bool)
    def on_button_setup_pressed(self):
        scenario = self.world_comboBox.currentText()

        # Disable buttons
        self.button_RI.setEnabled(False)
        self.button_setup.setEnabled(False)
        self.button_setup_exp.setEnabled(False)
        self.button_remove_robot.setEnabled(False)
        self.world_comboBox.setEnabled(False)
        self.button_start_sim.setEnabled(True)
        self.button_addRobot.setEnabled(False)
        self.button_record_rosbag.setEnabled(True)
        self.tabWidget.setEnabled(False)
        self.button_load_scenario.setEnabled(False)
        self.button_execute_task.setEnabled(False)

        # Get robot types to generate RVIZ file
        robot_list = []
        for i in range(0, self.num_robots):
            #if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
            #    robot_list.append('tiago')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
            #    robot_list.append('turtlebot')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'srd250':
            #    robot_list.append('srd250')
            robot_list.append(self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])
        file = RVIZFileGenerator(robot_list)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Launch gazebo world
        launch_world = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'setup_simulation.launch')])
        sys.argv.append('scenario:=' + scenario)
        launch_world.start()
        del sys.argv[2:len(sys.argv)]

        # Launch rosbag logger
        launch_logger = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rosbag_writer.launch')])
        sys.argv.append('num_robots:=' + str(self.num_robots))
        launch_logger.start()
        del sys.argv[2:len(sys.argv)]

        # Launch robots
        launch_robot_list = []
        for i in range(0, self.num_robots):
            launch_robot_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot.launch')]))

            # Get orientation from pose
            quaternion = Quaternion(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]

            # Get robot model
            #if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
            #    sys.argv.append('robot_model:=tiago_steel')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
            #    sys.argv.append('robot_model:=turtlebot')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'srd250':
            #    sys.argv.append('robot_model:=srd250')

            sys.argv.append('robot_model:=' + self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])

            # Set arguments for launch file
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('initial_pose_x:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][0]))
            sys.argv.append('initial_pose_y:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][1]))
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

            self.add_region_marker(self.tab_list[i].initial_pose, True)

            rospy.loginfo("server up")

        # Publish region marker
        self.add_region_marker(self.FTS.region_of_interest, False)
        self.ros_publisher.add_publisher('region_of_interest', MarkerArray, 1.0, self.region_pose_marker_array_msg)

    @Slot(bool)
    def on_button_setup_exp_pressed(self):
        scenario = self.world_comboBox.currentText()

        # Disable buttons
        self.button_RI.setEnabled(False)
        self.button_setup.setEnabled(False)
        self.button_setup_exp.setEnabled(False)
        self.button_remove_robot.setEnabled(False)
        self.world_comboBox.setEnabled(False)
        self.button_start_sim.setEnabled(True)
        self.button_addRobot.setEnabled(False)
        self.button_record_rosbag.setEnabled(True)
        self.tabWidget.setEnabled(False)
        self.button_load_scenario.setEnabled(False)
        self.button_execute_task.setEnabled(False)

        #Get robot types to generate RVIZ file
        robot_list = []
        for i in range(0, self.num_robots):
            #if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
            #    robot_list.append('tiago')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
            #    robot_list.append('turtlebot')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'srd250':
            #    robot_list.append('srd250')
            robot_list.append(self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])
        file = RVIZFileGenerator(robot_list)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Launch rosbag logger
        launch_logger = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rosbag_writer.launch')])
        sys.argv.append('num_robots:=' + str(self.num_robots))
        launch_logger.start()
        del sys.argv[2:len(sys.argv)]

        # Launch rviz
        launch_rviz = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rviz.launch')])
        launch_rviz.start()

        # Launch qualysis mapper
        launch_qualisys = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'qualisys_mapper.launch')])
        launch_qualisys.start()

        # Launch robots
        launch_robot_list = []
        for i in range(0, self.num_robots):
            launch_robot_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot_exp.launch')]))

            # Get orientation from pose
            quaternion = Quaternion(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]
            #if self.tab_list[i].robot_comboBox.currentText() == 'TiaGo':
            #    sys.argv.append('robot_model:=tiago_steel')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'Turtlebot':
            #    sys.argv.append('robot_model:=turtlebot')
            #elif self.tab_list[i].robot_comboBox.currentText() == 'srd250':
            #    sys.argv.append('robot_model:=srd250')

            sys.argv.append('robot_model:=' + self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])

            # Set arguments for launch file
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('initial_pose_x:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][0]))
            sys.argv.append('initial_pose_y:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][1]))
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

            self.add_region_marker(self.tab_list[i].initial_pose, True)

            rospy.loginfo("server up")

        # Publish region marker
        self.add_region_marker(self.FTS.region_of_interest, False)
        self.ros_publisher.add_publisher('region_of_interest', MarkerArray, 1.0, self.region_pose_marker_array_msg)

    @Slot(bool)
    def on_button_execute_task_pressed(self):
        print('saved task')
        env_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        stream = file(env_file, 'r')
        FTS = yaml.load(stream)
        stream.close()

        data = FTS
        robot_setup = {}

        # Launch ltl_planner
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_task_list = []
        for i in range(0, self.num_robots):
            robot_model = self.tab_list[i].robot_comboBox.currentText()
            initial_pose = self.tab_list[i].robot_comboBox_init.currentText()
            tasks = {'hard_task' : self.tab_list[i].robot_hard_task_input.text(), 'soft_task' : self.tab_list[i].robot_soft_task_input.text()}
            robot_setup.update({self.tab_list[i].robot_name : {'robot_model' : robot_model, 'initial_pose' : initial_pose, 'tasks' : tasks}})
            self.tab_list[i].soft_task_msg.data = self.tab_list[i].robot_soft_task_input.text()
            self.tab_list[i].hard_task_msg.data = self.tab_list[i].robot_hard_task_input.text()
            roslaunch_task_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'ltl_planner.launch')]))
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('agent_type:=' + self.tab_list[i].agent_type)
            sys.argv.append('scenario_file:=' + self.scenario_file)
            roslaunch_task_list[i].start()
            del sys.argv[2:len(sys.argv)]

        data.update({'Tasks' : robot_setup})
        data.update({'Map' : self.world_comboBox.currentText()})
        with codecs.open(env_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(data, outfile, default_flow_style=False)

    # Enable LTL-planner
    @Slot(bool)
    def on_button_start_sim_pressed(self):      
        start_msg = Bool()
        start_msg.data = True
        self.start_publisher.publish(start_msg)
        for i in range(0, self.num_robots):
            self.tab_list[i].simulation_started = True

    # Enable and disable logger
    @Slot(bool)
    def on_button_rosbag_clicked(self):
        if self.logger_active_msg.data == False:
            self.logger_active_msg.data = True
            self.button_record_rosbag.setText('Stop recording')
        elif self.logger_active_msg.data == True:
            self.logger_active_msg.data = False
            self.button_record_rosbag.setText('Continue recording')

    # Convert position message to position tuple
    def position_msg_to_tuple(self, position_msg):
        position = (position_msg.x, position_msg.y, position_msg.z)
        return position

    # Add robot tab
    @Slot(bool)
    def add_robot(self):
        # Add tab
        self.num_robots += 1
        self.tab_list.append(RobotTab(self.num_robots, self.robots))
        self.tabWidget.addTab(self.tab_list[self.num_robots-1], ('Robot ' + str(self.num_robots)))
        self.button_remove_robot.setEnabled(True)
        self.current_graphicsScene.addItem(self.tab_list[self.num_robots-1].initial_pose['start_' + str(self.num_robots).zfill(2)]['text_item'])

        # Set initial pose
        if self.num_robots > 1:
           for i in range(0, len(self.FTS.region_of_interest)):
               self.tab_list[self.num_robots-1].robot_comboBox_init.addItem(self.FTS.region_of_interest.keys()[i])
           self.tab_list[self.num_robots-1].robot_comboBox_init.model().sort(0)
        if len(self.current_graphicsScene.items_dict) > 0:
           self.tab_list[self.num_robots-1].init_pose_msg.position.x = self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[0]]['pose']['position'][0]
           self.tab_list[self.num_robots-1].init_pose_msg.position.y = self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[0]]['pose']['position'][1]
           self.tab_list[self.num_robots-1].init_pose_msg.position.z = self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[0]]['pose']['position'][2]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.w = self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[0]]['pose']['orientation'][0]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.x = self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[0]]['pose']['orientation'][1]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.y = self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[0]]['pose']['orientation'][2]
           self.tab_list[self.num_robots-1].init_pose_msg.orientation.z = self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[0]]['pose']['orientation'][3]

           self.current_graphicsScene.items_dict['r01']['ellipse_item'].setBrush(QBrush(QColor('green')))
           rect = self.current_graphicsScene.items_dict['r01']['ellipse_item'].rect()
           point = rect.topLeft()
           self.tab_list[self.num_robots-1].initial_pose['start_' + str(self.num_robots).zfill(2)]['text_item'].setPos(point.x() - 11, point.y() - 22)

        # Connect initial pose comboBox with set_init_pose_id function
        self.tab_list[self.num_robots-1].robot_comboBox_init.signalIndexChanged.connect(self.set_init_pose_id)

        # Add plan topics and subscriber
        self.prefix_plan_topic_list.append('/' + self.tab_list[self.num_robots-1].robot_name + '/prefix_plan')
        self.prefix_plan_subscriber_list.append(ROS_Subscriber(self.prefix_plan_topic_list[self.num_robots-1], PoseArray, self.prefix_callback))
        self.sufix_plan_topic_list.append('/' + self.tab_list[self.num_robots-1].robot_name + '/sufix_plan')
        self.sufix_plan_subscriber_list.append(ROS_Subscriber(self.sufix_plan_topic_list[self.num_robots-1], PoseArray, self.sufix_callback))

        self.prefix_plan_subscriber_list[self.num_robots-1].received.connect(self.received_prefix)
        self.sufix_plan_subscriber_list[self.num_robots-1].received.connect(self.received_sufix)

        # Add current goal subscriber
        self.current_goal_topic_list.append('/' + self.tab_list[self.num_robots-1].robot_name + '/move_base/goal')
        self.current_goal_subscriber_list.append(ROS_Subscriber(self.current_goal_topic_list[self.num_robots-1], MoveBaseActionGoal, self.goal_callback))
        self.current_goal_subscriber_list[self.num_robots-1].received.connect(self.received_goal)

    # Remove last robot
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

            self.current_graphicsScene.removeItem(self.tab_list[self.num_robots].initial_pose['start_' + str(self.num_robots+1).zfill(2)]['text_item'])

            for i in range(0, len(self.current_graphicsScene.items_dict)):
                self.current_graphicsScene.items_dict[self.current_graphicsScene.items_dict.keys()[i]]['ellipse_item'].setBrush(QBrush(QColor('red')))

            for i in range(0, self.num_robots):
                self.current_graphicsScene.items_dict[self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['label']]['ellipse_item'].setBrush(QBrush(QColor('green')))
                rect = self.current_graphicsScene.items_dict[self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['label']]['ellipse_item'].rect()
                point = rect.topLeft()
                self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'].setPos(point.x() - 11, point.y() - 22)

            self.tabWidget.removeTab(self.num_robots)
            del self.tab_list[self.num_robots]

            if self.num_robots == 1:
                self.button_remove_robot.setEnabled(False)

    # Load scenario from a yaml file
    @Slot(bool)
    def load_scenario(self):
        directory = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS')
        File_dialog = QFileDialog(directory=directory, filter='.yaml')
        scenario_file = File_dialog.getOpenFileName()
        stream = file(scenario_file[0], 'r')
        data = yaml.load(stream)

        # Load map
        self.world_comboBox.setCurrentIndex(self.world_comboBox.findText(data['Map']))

        # Load FTS
        # Sort the keys is needed for the edge matrix in the map_dialog
        self.FTS.region_of_interest = data['FTS']
        sorted_keys = self.FTS.region_of_interest.keys()
        sorted_keys.sort()
        stream.close()

        #print(self.FTS.region_of_interest)

        self.scenario_file = scenario_file[0]

        arrow_length = 50

        # Add all region of interest to graphics scene
        for i in range(0, len(self.FTS.region_of_interest)):
            region_string = 'r' + str(i+1).zfill(2)
            pixel_coords = self.current_graphicsScene.worldToPixel(self.FTS.region_of_interest[sorted_keys[i]]['pose']['position'])
            self.current_graphicsScene.add_ROI(pixel_coords)

            quaternion = Quaternion(self.FTS.region_of_interest[sorted_keys[i]]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]
            end_point = QPointF(pixel_coords.x() + arrow_length * cos(theta), pixel_coords.y() - arrow_length * sin(theta))
            arrow = self.current_graphicsScene.addArrow(pixel_coords, end_point)
            self.current_graphicsScene.items_dict[region_string]['arrow'] = arrow


        # Add all edges to graphics scene
        for i in range(0, len(self.FTS.region_of_interest)):
            for j in range(0, len(self.FTS.region_of_interest[sorted_keys[i]]['edges'])):
                index = sorted_keys.index(self.FTS.region_of_interest[sorted_keys[i]]['edges'][j]['target'])
                if i < index:
                    if (str(i+1) + '-' + str(index+1)) not in self.current_graphicsScene.line_dict.keys():
                        self.current_graphicsScene.add_edge(i+1, index+1)
                else:
                    if (str(index+1) + '-' + str(i+1)) not in self.current_graphicsScene.line_dict.keys():
                        self.current_graphicsScene.add_edge(index+1, i+1)

        # Load robot tabs
        robot_tabs = data['Tasks']

        for i in range(0, len(self.FTS.region_of_interest)):
            self.tab_list[self.num_robots-1].robot_comboBox_init.addItem(self.FTS.region_of_interest.keys()[i])
        self.tab_list[self.num_robots-1].robot_comboBox_init.model().sort(0)


        for i in range(0, len(robot_tabs)):
            if i > 0:
                self.add_robot()
            self.tab_list[i].robot_comboBox.setCurrentIndex(self.tab_list[i].robot_comboBox.findText(robot_tabs['robot' + str(i+1)]['robot_model']))
            self.tab_list[i].robot_comboBox_init.setCurrentIndex(self.tab_list[i].robot_comboBox_init.findText(robot_tabs['robot' + str(i+1)]['initial_pose']))
            self.tab_list[i].robot_hard_task_input.setText(robot_tabs['robot' + str(i+1)]['tasks']['hard_task'])
            self.tab_list[i].robot_soft_task_input.setText(robot_tabs['robot' + str(i+1)]['tasks']['soft_task'])

        self.button_execute_task.setEnabled(True)

    # Add region markers for RVIZ
    # Red marker: General ROI
    # Green marker : Start ROI
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

class FTS:
    def __init__(self):
        #self.region_list = []
        self.region_of_interest = {}

    def add_region(self, label, edges = list(), pose = dict()):
        #self.region_list.append(label)
        self.region_of_interest.update({label : {'edges' : edges, 'pose' : pose}})

    def add_edge(self, label, target_label, cost):
        self.region_of_interest[label]['edges'].append({'cost' : cost, 'target' : target_label})

    def remove_edge(self, label, target_label):
        for i in range(0, len(self.region_of_interest[label]['edges'])):
            if self.region_of_interest[label]['edges'][i]['target'] == target_label:
                index = i
        del self.region_of_interest[label]['edges'][i]
