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
from inspect import currentframe, getframeinfo

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem, QVBoxLayout, QComboBox, QLineEdit, QTextBrowser, QGridLayout, QFileDialog
from python_qt_binding.QtCore import QTimer, Slot, pyqtSlot, QSignalMapper, QRectF, QPointF, Qt
from python_qt_binding.QtGui import QImageReader, QImage, QMouseEvent, QCursor, QBrush, QColor, QPixmap, QTransform, QFont

from rqt_simulation.map_dialog import Map_dialog
from rqt_simulation.change_FTS_dialog import Change_FTS_dialog
from rqt_simulation.MapGraphicsScene import MapGraphicsScene
from rqt_simulation.ROS_Subscriber import ROS_Subscriber
from rqt_simulation.CustomComboBox import CustomComboBox
from rqt_simulation.RVIZFileGenerator import RVIZFileGenerator
from rqt_simulation.ROS_Publisher import ROS_Publisher
from rqt_simulation.RobotTab import RobotTab
from rqt_simulation.WidgetUtiles import WidgetUtiles

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

        self.cf = currentframe()
        self.filename = getframeinfo(self.cf).filename

        # Load ui file
        try:
            ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'SimulationPlugin.ui')
            loadUi(ui_file, self)
        except:
            print('In file ' + self.filename + ' at line ' + str(self.cf.f_lineno) + ': Error while loading ui file')
            exit()

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
        self.button_change_FTS.clicked.connect(self.on_button_change_FTS_pressed)

        # Disable buttons
        self.button_setup.setEnabled(False)
        self.button_setup_exp.setEnabled(False)
        self.button_remove_robot.setEnabled(False)
        self.button_start_sim.setEnabled(False)
        self.button_execute_task.setEnabled(False)
        self.button_record_rosbag.setEnabled(False)

        # Load configuration available robots and worlds
        try:
            config_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'gui_config.yaml')
            stream = file(config_file, 'r')
        except:
            print('In file ' + self.filename + ' at line ' + str(self.cf.f_lineno) + ': Error while loading gui configuration file')
            exit()
        data = yaml.load(stream)

        self.robots = data['Robots']
        worlds = data['Worlds']

        # Initialize FTS
        self.FTS = FTS()

        self.widget_utiles = WidgetUtiles()

        # Get default scenario file
        try:
            self.scenario_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        except:
            print('In file ' + self.filename + ' at line ' + str(self.cf.f_lineno) + ': Error while loading env_GUI.yaml file')
            exit()

        # Variables for ROS Publisher
        self.ros_publisher = ROS_Publisher()

        # Robot tab variables
        self.num_robots = 0
        self.tab_list = []

        # Add the select world combo box
        self.world_comboBox.addItems(worlds)

        # Variable setting rosbag writer subscribers active
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
        transform = self.current_graphicsScene.scale_map(self.graphicsView_main, self.scenario)
        self.graphicsView_main.setTransform(transform)

        # Add robot tab
        self.add_robot()

        # Publisher to set ltl_planner active
        self.start_publisher = rospy.Publisher('/planner_active', Bool, queue_size = 1)

        # Publisher to set logger active
        self.logger_active_msg = Bool()
        self.logger_active_msg.data = False
        self.ros_publisher.add_publisher('/logger_active', Bool, 1.0, self.logger_active_msg)

        # Counter for marker id counter
        #self.marker_id_counter = 0

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
        transform = self.current_graphicsScene.scale_map(self.graphicsView_main, self.scenario)
        self.graphicsView_main.setTransform(transform)

        # Reset initial pose and plan textboxes
        for i in range(0, self.num_robots):
            self.tab_list[i].robot_comboBox_init.clear()
            self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'] = QGraphicsTextItem('start_' + str(i+1).zfill(2))
            self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'].setPos(0.0, 0.0)
            self.current_graphicsScene.addItem(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'])
            self.tab_list[i].robot_sufix_textbox.clear()
            self.tab_list[i].robot_prefix_textbox.clear()

        self.prefix_string = ''
        self.sufix_string = ''

        # Reinitialize FTS
        self.FTS = FTS()

        # Load new map
        try:
            self.scenario_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        except:
            print('In file ' + self.filename + ' at line ' + str(self.cf.f_lineno) + ': Error while loading env_GUI.yaml file')
            exit()

        # Reinitialize ROI marker msg
        self.region_pose_marker_array_msg = MarkerArray()


    # Callback for prefix from ltl_planner
    def prefix_callback(self, msg, source):
        self.prefix_string = ''
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
        self.button_setup.setEnabled(True)
        self.button_setup_exp.setEnabled(True)

    # Callback for sufix from ltl_planner
    def sufix_callback(self, msg, source):
        self.sufix_string = ''
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
       # self.sufix_string = ''

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

        # Add initial poses to comboBox
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
            self.tab_list[id-1].build_init_pose_msg(id)

            # Update graphics scene
            for i in range(0, len(self.current_graphicsScene.items_dict)):
                self.current_graphicsScene.items_dict[self.current_graphicsScene.items_dict.keys()[i]]['ellipse_item'].setBrush(QBrush(QColor('red')))

            for i in range(0, self.num_robots):
                self.current_graphicsScene.items_dict[self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['label']]['ellipse_item'].setBrush(QBrush(QColor('green')))
                rect = self.current_graphicsScene.items_dict[self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['label']]['ellipse_item'].rect()
                point = rect.topLeft()
                self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['text_item'].setPos(point.x() - 11, point.y() - 22)

    # Setup simulation, gazebo and RVIZ
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
        #self.tabWidget.setEnabled(False)
        self.button_load_scenario.setEnabled(False)
        self.button_execute_task.setEnabled(False)

        # Get robot types to generate RVIZ file
        robot_list = []
        tf_prefixes = []
        for i in range(0, self.num_robots):
            robot_list.append(self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])
            tf_prefixes.append(self.tab_list[i].robot_name)

        file = RVIZFileGenerator(robot_list, tf_prefixes)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Launch gazebo world
        self.widget_utiles.launch_gazebo(scenario)

        # Launch rosbag logger
        self.widget_utiles.launch_logger()

        # Launch robots
        launch_robot_list = []
        for i in range(0, self.num_robots):
            launch_robot_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot.launch')]))

            # Get orientation from pose
            quaternion = Quaternion(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]

            sys.argv.append('robot_model:=' + self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])

            # Set arguments for launch file
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('initial_pose_x:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][0]))
            sys.argv.append('initial_pose_y:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][1]))
            sys.argv.append('initial_pose_a:=' + str(theta))
            sys.argv.append('scenario:=' + scenario)

            launch_robot_list[i].start()
            navigation = actionlib.SimpleActionClient('/' + self.tab_list[i].robot_name + '/move_base', MoveBaseAction)

            del sys.argv[2:len(sys.argv)]

            self.FTS.add_region_marker(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)], 'start_' + str(i+1).zfill(2), True)

            rospy.loginfo("server up")

        # Publish region marker
        for i in range(0, len(self.FTS.region_of_interest)):
            self.FTS.add_region_marker(self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[i]], self.FTS.region_of_interest.keys()[i], False)

    # Setup experiment, RVIZ
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
        #self.tabWidget.setEnabled(False)
        self.button_load_scenario.setEnabled(False)
        self.button_execute_task.setEnabled(False)

        #Get robot types to generate RVIZ file
        robot_list = []
        tf_prefixes = []
        for i in range(0, self.num_robots):
            robot_list.append(self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])
            tf_prefixes.append(self.tab_list[i].robot_name)

        file = RVIZFileGenerator(robot_list, tf_prefixes)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Launch rosbag logger
        self.widget_utiles.launch_logger()

        # Launch rviz
        launch_rviz = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rviz.launch')])
        launch_rviz.start()

        # Load transformation between map and qualisys
        if self.tab_list[i].robot_localization_checkBox.checkState() == 2:
            # Launch qualysis mapper
            self.widget_utiles.set_qualisys_args(self.current_graphicsScene.tf_qualisys_to_map)
            self.widget_utiles.launch_qualisys()

        # Launch robots
        launch_robot_list = []
        for i in range(0, self.num_robots):
            launch_robot_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot_exp.launch')]))

            # Get orientation from pose
            quaternion = Quaternion(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]

            sys.argv.append('robot_model:=' + self.robots['Models'][self.tab_list[i].robot_comboBox.currentText()]['robot_model'])

            # Set arguments for launch file
            sys.argv.append('robot_name:=' + self.tab_list[i].robot_name)
            sys.argv.append('initial_pose_x:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][0]))
            sys.argv.append('initial_pose_y:=' + str(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose']['position'][1]))
            sys.argv.append('initial_pose_a:=' + str(theta))
            sys.argv.append('scenario:=' + scenario)
            if self.tab_list[i].robot_localization_checkBox.checkState() == 2:
                sys.argv.append('use_qualisys:=true')
            else:
                sys.argv.append('use_qualisys:=false')

            self.widget_utiles.set_qualisys_args(self.current_graphicsScene.tf_qualisys_to_map)

            launch_robot_list[i].start()

            del sys.argv[2:len(sys.argv)]

            self.FTS.add_region_marker(self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)], 'start_' + str(i+1).zfill(2), True)

            rospy.loginfo("server up")

        # Publish region marker
        for i in range(0, len(self.FTS.region_of_interest)):
            self.FTS.add_region_marker(self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[i]], self.FTS.region_of_interest.keys()[i], False)

    # Synthesize the tasks and start LTL Planner
    @Slot(bool)
    def on_button_execute_task_pressed(self):
        print('saved task')
        env_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        stream = file(env_file, 'r')
        FTS = yaml.load(stream)
        stream.close()

        data = FTS
        robot_setup = {}

        self.init_planner_publisher_and_subscriber(self.num_robots)

        # Launch ltl_planner
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_task_list = []
        for i in range(0, self.num_robots):

            robot_model = self.tab_list[i].robot_comboBox.currentText()
            initial_pose = self.tab_list[i].robot_comboBox_init.currentText()
            tasks = {'hard_task' : self.tab_list[i].robot_hard_task_input.text(), 'soft_task' : self.tab_list[i].robot_soft_task_input.text()}
            if self.tab_list[i].robot_localization_checkBox.checkState() == 2:
                use_qualisys = True
            else:
                use_qualisys = False
            robot_setup.update({self.tab_list[i].robot_name : {'robot_model' : robot_model, 'use_qualisys' : use_qualisys, 'initial_pose' : initial_pose, 'tasks' : tasks}})
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
        #self.tab_list[self.num_robots-1].signalRobotNameChanged.connect(self.robot_name_changed)
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


    # Initilaize Planner Publisher and Subscriber
    def init_planner_publisher_and_subscriber(self, num_robots):
        # Remove plan topics and subscriber
        self.prefix_plan_topic_list = []
        self.prefix_plan_subscriber_list = []
        self.sufix_plan_topic_list = []
        self.sufix_plan_subscriber_list = []

        # Remove current goal subscriber
        self.current_goal_topic_list = []
        self.current_goal_subscriber_list = []

        for i in range(0, num_robots):
            self.tab_list[i].start_publisher_and_subscriber()
            # Add plan topics and subscriber
            self.prefix_plan_topic_list.append('/' + self.tab_list[i].robot_name + '/prefix_plan')
            self.prefix_plan_subscriber_list.append(ROS_Subscriber(self.prefix_plan_topic_list[i], PoseArray, self.prefix_callback))
            self.sufix_plan_topic_list.append('/' + self.tab_list[i].robot_name + '/sufix_plan')
            self.sufix_plan_subscriber_list.append(ROS_Subscriber(self.sufix_plan_topic_list[i], PoseArray, self.sufix_callback))

            self.prefix_plan_subscriber_list[i].received.connect(self.received_prefix)
            self.sufix_plan_subscriber_list[i].received.connect(self.received_sufix)

            # Add current goal subscriber
            self.current_goal_topic_list.append('/' + self.tab_list[i].robot_name + '/move_base/goal')
            self.current_goal_subscriber_list.append(ROS_Subscriber(self.current_goal_topic_list[i], MoveBaseActionGoal, self.goal_callback))
            self.current_goal_subscriber_list[i].received.connect(self.received_goal)



    # Remove last robot
    @Slot(bool)
    def remove_robot(self):
        if self.num_robots > 1:
            self.num_robots = self.num_robots - 1

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

    @Slot(bool)
    def on_button_change_FTS_pressed(self):
        # Start change FTS dialog
        change_FTS_dialog = Change_FTS_dialog(self.current_graphicsScene, self.FTS)
        change_FTS_dialog.exec_()

    # Load scenario from a yaml file
    @Slot(bool)
    def load_scenario(self):
        # Start file dialog GUI
        directory = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS')
        File_dialog = QFileDialog(directory=directory, filter='.yaml')
        scenario_file = File_dialog.getOpenFileName()
        try:
            stream = file(scenario_file[0], 'r')
        except:
            print('In file ' + self.filename + ' at line ' + str(self.cf.f_lineno) + ': Error no valid scenario file')
            return False

        data = yaml.load(stream)

        # Remove all robots
        for i in range(0, len(self.tab_list) - 1):
            self.remove_robot()

        # Reset map
        self.reset()

        # Load map
        self.world_comboBox.setCurrentIndex(self.world_comboBox.findText(data['Map']))

        # Load FTS
        # Sort the keys is needed for the edge matrix in the map_dialog
        self.FTS.region_of_interest = data['FTS']
        sorted_keys = self.FTS.region_of_interest.keys()
        sorted_keys.sort()
        stream.close()

        self.scenario_file = scenario_file[0]

        self.current_graphicsScene.load_graphic_from_FTS(self.FTS)

        # Load robot tabs
        robot_tabs = data['Tasks']

        for i in range(0, len(self.FTS.region_of_interest)):
            self.tab_list[self.num_robots-1].robot_comboBox_init.addItem(self.FTS.region_of_interest.keys()[i])
        self.tab_list[self.num_robots-1].robot_comboBox_init.model().sort(0)

        for i in range(0, len(robot_tabs)):
            if i > 0:
                self.add_robot()
            self.tab_list[i].robot_name_input.setText(robot_tabs.keys()[i])
            self.tab_list[i].robot_comboBox.setCurrentIndex(self.tab_list[i].robot_comboBox.findText(robot_tabs[robot_tabs.keys()[i]]['robot_model']))
            self.tab_list[i].robot_comboBox_init.setCurrentIndex(self.tab_list[i].robot_comboBox_init.findText(robot_tabs[robot_tabs.keys()[i]]['initial_pose']))
            self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['pose'] = self.FTS.region_of_interest[self.tab_list[i].robot_comboBox_init.currentText()]['pose']
            self.tab_list[i].initial_pose['start_' + str(i+1).zfill(2)]['label'] = self.tab_list[i].robot_comboBox_init.currentText()
            if robot_tabs[robot_tabs.keys()[i]]['use_qualisys']:
                self.tab_list[i].robot_localization_checkBox.setCheckState(Qt.Checked)
            self.tab_list[i].robot_hard_task_input.setText(robot_tabs[robot_tabs.keys()[i]]['tasks']['hard_task'])
            self.tab_list[i].robot_soft_task_input.setText(robot_tabs[robot_tabs.keys()[i]]['tasks']['soft_task'])
            #self.tab_list[i].robot_name_changed()

        self.button_execute_task.setEnabled(True)


class FTS:
    def __init__(self):
        #self.region_list = []
        self.region_of_interest = {}
        self.marker_id_counter = 0
        self.region_pose_marker_array_msg = MarkerArray()
        self.ros_publisher = ROS_Publisher()
        self.ros_publisher.add_publisher('region_of_interest', MarkerArray, 1.0, self.region_pose_marker_array_msg)

    def add_region(self, label, edges = list(), pose = dict()):
        self.region_of_interest.update({label : {'edges' : edges, 'pose' : pose, 'propos' : [label]}})

    def add_edge(self, label, target_label, cost):
        self.region_of_interest[label]['edges'].append({'cost' : cost, 'target' : target_label})

    def remove_edge(self, label, target_label):
        for i in range(0, len(self.region_of_interest[label]['edges'])):
            if self.region_of_interest[label]['edges'][i]['target'] == target_label:
                del self.region_of_interest[label]['edges'][i]

    def add_propos(self, label, ap):
        self.region_of_interest[label]['propos'].append(ap)

    def remove_propos(self, label, ap):
        for i in range(0, len(self.region_of_interest[label]['propos'])):
            if self.region_of_interest[label]['propos'][i] == ap:
                del self.region_of_interest[label]['propos'][i]

    def load_FTS(self):
        # Start file dialog
        directory = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS')
        File_dialog = QFileDialog(directory=directory, filter='.yaml')
        FTS_file = File_dialog.getOpenFileName()
        stream = file(FTS_file[0], 'r')
        data = yaml.load(stream)
        stream.close()

        # Load FTS
        self.region_of_interest = {}
        self.region_of_interest = data['FTS']



    # Add region markers for RVIZ
    # Red marker: General ROI
    # Green marker : Start ROI
    def add_region_marker(self, region, label, initial):

        pose_marker = Pose()
        pose_text = Pose()
        self.region_pose_marker = Marker()
        self.region_pose_marker_label = Marker()

        self.region_pose_marker.pose = pose_marker
        self.region_pose_marker_label.pose = pose_text

        pose_marker.position.x = region['pose']['position'][0]
        pose_marker.position.y = region['pose']['position'][1]
        pose_text.position.x = region['pose']['position'][0]
        pose_text.position.y = region['pose']['position'][1]

        if initial:
            self.region_pose_marker_label.text = label
            self.region_pose_marker.color.r = 0.0
            self.region_pose_marker.color.g = 0.5
            self.region_pose_marker.color.b = 0.0
            self.region_pose_marker.pose.position.z = 0.01
            self.region_pose_marker_label.pose.position.z = 1.5
            self.region_pose_marker.scale.x = 0.5
            self.region_pose_marker.scale.y = 0.5
        else:
            self.region_pose_marker_label.text = label
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
