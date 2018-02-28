#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import yaml
import codecs
import roslaunch
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem
from python_qt_binding.QtCore import QTimer, Slot, pyqtSlot, QSignalMapper, QRectF, QPointF
from python_qt_binding.QtGui import QImageReader, QImage, QMouseEvent, QCursor, QBrush, QColor, QPixmap, QTransform

from .map_dialog import Map_dialog
from .initial_pose import Initial_pose
from rqt_simulation.MapGraphicsScene import MapGraphicsScene
from rqt_simulation.ROS_Subscriber import ROS_Subscriber

from ltl_tools.planner import ltl_planner

from ltl_tools.ts import MotionFts, ActionModel, MotActModel


class SimulationWidget(QWidget):

    def __init__(self):
        print('constructor')
        super(SimulationWidget, self).__init__()
        self.setObjectName('SimulationWidget')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'SimulationPlugin.ui')
        loadUi(ui_file, self)

        self.button_RI.pressed.connect(self.on_button_RI_pressed)
        self.button_setup.clicked.connect(self.on_button_setup_pressed)
        self.button_execute_task.clicked.connect(self.on_button_execute_task_pressed)
        self.comboBox_robot1.currentIndexChanged.connect(self.on_comboBox_robot1_indexChanged)
        self.world_comboBox.currentIndexChanged.connect(self.initialization)
        self.comboBox_init_pose1.currentIndexChanged.connect(self.set_init_pose)
        self.button_start_sim.clicked.connect(self.on_button_start_sim_pressed)

        #self.prefix_plan_subscriber = rospy.Subscriber('/tiago1/prefix_plan', PoseArray, self.prefix_callback)
        self.prefix_plan_subscriber = ROS_Subscriber('/tiago1/prefix_plan', PoseArray, self.prefix_callback)
        self.sufix_plan_subscriber = ROS_Subscriber('/tiago1/sufix_plan', PoseArray, self.sufix_callback)

        self.prefix_plan_subscriber.received.connect(self.received_prefix)
        self.sufix_plan_subscriber.received.connect(self.received_sufix)

        self.start_publisher = rospy.Publisher('tiago1/planner_active', Bool, queue_size = 1)

        self.initialization()

        #self.graphicsView_main.scale(0.5, 0.5)

        print('constructor loaded')

    def initialization(self):
        #self.button_setup.setEnabled(False)
        self.comboBox_robot2.setEnabled(False)
        #self.button_RI.setEnabled(False)
        self.hard_task_input.setEnabled(False)
        self.soft_task_input.setEnabled(False)
        self.comboBox_init_pose1.setEnabled(False)
        self.comboBox_init_pose2.setEnabled(False)
        self.current_graphicsScene = MapGraphicsScene()
        self.current_graphicsScene.clear()
        self.graphicsView_main.setScene(self.current_graphicsScene)
        self.ellipse_items_RI = []
        self.ellipse_items_labels_RI = []
        self.initial_pose_labels = [QGraphicsTextItem('start_01')]
        self.comboBox_robot1.setCurrentIndex(0)
        self.comboBox_robot2.setCurrentIndex(0)
        self.initial_pose = {}
        self.region_of_interest = {}
        self.comboBox_init_pose1.clear()
        self.comboBox_init_pose2.clear()
        self.line_dict = {}
        self.prefix_string = ''
        self.textBrowser_prefix.insertPlainText('Prefix: ')
        self.sufix_string = ''
        self.textBrowser_sufix.insertPlainText('Sufix: ')

        self.scenario = self.world_comboBox.currentText()
        map_yaml = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, 'map.yaml')
        self.loadConfig(map_yaml)
        if self.scenario == 'pal_office':
            map = 'map.pgm'
        else:
            map = 'map.png'

        map_file = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, map)
        pixmap = QPixmap(map_file)
        mapSize = pixmap.size()
        self.current_graphicsScene.addPixmap(pixmap)

        self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + mapSize.height())
        self.current_graphicsScene.addCoordinateSystem(self.worldOrigin, 0.0)

        rectF = self.graphicsView_main.geometry()

        if (float(rectF.width())/mapSize.width() < float(rectF.height())/mapSize.height()):
            scale = float(rectF.width())/mapSize.width()
        else:
            scale = float(rectF.height())/mapSize.height()

        #matrix = QMatrix(scale, 0, 0.0, scale, 0, 0)
        transform = QTransform(scale, 0, 0.0, scale, 0, 0)
        #self.graphicsView_main.setMatrix(matrix)
        self.graphicsView_main.setTransform(transform)

        self.region_pose_marker_array_msg = MarkerArray()

        self.id_counter = 0
        self.publisher_dict = {}
        self._timeout_mapper = QSignalMapper(self)
        self._timeout_mapper.mapped[int].connect(self.publish_once)

        self.marker_id_counter = 0

    def prefix_callback(self, msg):
        for n in msg.poses:
            #print self.position_msg_to_tuple(n.position)
            for i in range(0, len(self.region_of_interest)):
                #print self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position']
                if self.position_msg_to_tuple(n.position) == self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position']:
                    self.prefix_string =  self.prefix_string + self.region_of_interest.keys()[i] + ' --> '
                    #print self.prefix_string
        self.prefix_plan_subscriber.received.emit(True)

    @Slot(bool)
    def received_prefix(self):
        self.textBrowser_prefix.insertPlainText(self.prefix_string)

    def sufix_callback(self, msg):
        for n in msg.poses:
            for i in range(0, len(self.region_of_interest)):
                if self.position_msg_to_tuple(n.position) == self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position']:
                    self.sufix_string = self.sufix_string + self.region_of_interest.keys()[i] + ' --> '
        self.sufix_plan_subscriber.received.emit(True)

    @Slot(bool)
    def received_sufix(self):
        self.textBrowser_sufix.insertPlainText(self.sufix_string)

    @Slot(bool)
    def on_button_RI_pressed(self):
        self.button_RI.setEnabled(True)
        graphicScene_item = self.current_graphicsScene.items()
        print(len(graphicScene_item))
        if len(graphicScene_item) > 9:
            for i in range(0, len(self.ellipse_items_RI)):
                self.current_graphicsScene.removeItem(self.ellipse_items_RI[i])
                self.current_graphicsScene.removeItem(self.ellipse_items_labels_RI[i])
            for i in range(0, len(self.initial_pose_labels)):
                self.current_graphicsScene.removeItem(self.initial_pose_labels[i])
            for i in range(0, len(self.line_dict)):
                self.current_graphicsScene.removeItem(self.line_dict[self.line_dict.keys()[i]])
        self.comboBox_init_pose1.clear()
        self.comboBox_init_pose2.clear()

        map_dialog = Map_dialog(self.world_comboBox.currentText(), self.current_graphicsScene)
        map_dialog.exec_()
        self.ellipse_items_RI = map_dialog.ellipse_items
        self.ellipse_items_labels_RI = map_dialog.ellipse_items_labels
        self.region_of_interest = map_dialog.region_of_interest
        self.pixel_coords = map_dialog.pixel_coords_list
        self.region_list = map_dialog.region_list
        self.add_region_marker(self.region_of_interest, False)
        self.line_dict = map_dialog.line_dict
        if len(self.ellipse_items_RI) > 0:
            for i in range(0, len(self.region_of_interest)):
                self.comboBox_init_pose1.addItem(self.region_of_interest.keys()[i])
                self.comboBox_init_pose2.addItem(self.region_of_interest.keys()[i])
            self.comboBox_init_pose1.model().sort(0)
            self.comboBox_init_pose2.model().sort(0)
            self.hard_task_input.setEnabled(True)
            self.soft_task_input.setEnabled(True)
            self.current_graphicsScene.addItem(self.initial_pose_labels[0])
            self.comboBox_init_pose1.setEnabled(True)
            if self.comboBox_robot2.currentIndex() != 0:
                self.comboBox_init_pose2.setEnabled(True)

        print('works')

    @Slot(int)
    def on_comboBox_robot1_indexChanged(self):
        print(self.comboBox_robot1.currentIndex())
        if self.comboBox_robot1.currentIndex() == 0:
            self.comboBox_robot2.setCurrentIndex(0)
            self.comboBox_robot2.setEnabled(False)
        else:
            self.comboBox_robot2.setEnabled(True)

    @Slot(bool)
    def set_init_pose(self):
        if self.comboBox_init_pose1.count() > 0:
            self.initial_pose['start_01'] = self.region_of_interest[self.comboBox_init_pose1.currentText()]['pose']['position']
            print(self.region_list)

            index = self.region_list.index(self.comboBox_init_pose1.currentText())
            for i in range(0, len(self.region_list)):
                if index == i:
                    self.ellipse_items_RI[i].setBrush(QBrush(QColor('green')))
                    rect = self.ellipse_items_RI[i].rect()
                    point = rect.topLeft()
                    self.initial_pose_labels[0].setPos(point.x() - 11, point.y() - 22)

                else:
                    self.ellipse_items_RI[i].setBrush(QBrush(QColor('red')))


    @Slot(bool)
    def on_button_setup_pressed(self):
        scenario = self.world_comboBox.currentText()
        self.button_RI.setEnabled(False)
        self.button_setup.setEnabled(False)
        self.comboBox_robot1.setEnabled(False)
        self.comboBox_robot2.setEnabled(False)
        self.world_comboBox.setEnabled(False)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)


        launch_world = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'setup_simulation.launch')])
        sys.argv.append('scenario:=' + scenario)
        print(sys.argv)
        launch_world.start()


        launch_robot_1 = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot.launch')])
        sys.argv.append('robot_model:=tiago_steel')
        sys.argv.append('robot_name:=tiago1')
        sys.argv.append('initial_pose_x:=' + str(self.initial_pose['start_01'][0]))
        sys.argv.append('initial_pose_y:=' + str(self.initial_pose['start_01'][1]))
        sys.argv.append('initial_pose_a:=0.0')
        print(sys.argv)
        launch_robot_1.start()

        del sys.argv[2:len(sys.argv)]
        print(sys.argv)

        #launch_robot_2 = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'robot.launch')])
        #sys.argv.append('robot_model:=tiago_steel')
        #sys.argv.append('robot_name:=tiago2')
        #sys.argv.append('initial_pose_x:=' + str(self.initial_pose['r02'][0]))
        #sys.argv.append('initial_pose_y:=' + str(self.initial_pose['r02'][1]))
        #sys.argv.append('initial_pose_a:=0.0')
        #print(sys.argv)
        #launch_robot_2.start()

        self.add_region_marker(self.initial_pose, True)
        self.add_publisher('region_of_interest', MarkerArray, 1.0, self.region_pose_marker_array_msg)

    @Slot(bool)
    def on_button_execute_task_pressed(self):
        print('saved task')
        print(self.hard_task_input.text())
        print(self.soft_task_input.text())
        task_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'task', 'task.yaml')
        data = dict(
                    hard_task = self.hard_task_input.text(),
                    soft_task = self.soft_task_input.text())
        with codecs.open(task_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(data, outfile, default_flow_style=False)


        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_task = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'ltl_planner.launch')])

        sys.argv.append('robot_name:=tiago1')
        launch_task.start()
        self.textBrowser_prefix.insertPlainText(self.prefix_string)

    @Slot(bool)
    def on_button_start_sim_pressed(self):
        start_msg = Bool()
        start_msg.data = True
        self.start_publisher.publish(start_msg)

    @Slot(int)
    def publish_once(self, publisher_id):
        publisher = self.publisher_dict.get(publisher_id, None)
        if publisher is not None:
            publisher['publisher'].publish(publisher['message'])

    def add_publisher(self, topic, type, rate, msg):

        publisher = {}
        publisher['publisher_id'] = self.id_counter
        publisher['message'] = msg
        publisher['publisher'] = rospy.Publisher(topic, type, queue_size=1)
        self.publisher_dict[publisher['publisher_id']] = publisher
        #self.publisher_dict['publisher_id'].update({'publisher_id' : self.id_counter})
        print(self.publisher_dict)
        #self.publisher_dict.update({'publisher' : rospy.Publisher(topic, type, queue_size=1)})
        publisher['timer'] = QTimer(self)
        self._timeout_mapper.setMapping(publisher['timer'], publisher['publisher_id'])
        publisher['timer'].timeout.connect(self._timeout_mapper.map)
        publisher['timer'].start(int(1000.0 / rate))
        self.id_counter += 1

    def position_msg_to_tuple(self, position_msg):
        position = (position_msg.x, position_msg.y, position_msg.z)
        return position



    def add_region_marker(self, region, initial):

        for i in range(0, len(region)):
            pose_marker = Pose()
            pose_text = Pose()
            self.region_pose_marker = Marker()
            self.region_pose_marker_label = Marker()

            self.region_pose_marker.pose = pose_marker
            self.region_pose_marker_label.pose = pose_text

            if initial:
                pose_marker.position.x = region[region.keys()[i]][0]
                pose_marker.position.y = region[region.keys()[i]][1]
                pose_text.position.x = region[region.keys()[i]][0]
                pose_text.position.y = region[region.keys()[i]][1]
                self.region_pose_marker_label.text = region.keys()[0]
                self.region_pose_marker.color.r = 0.0
                self.region_pose_marker.color.g = 0.5
                self.region_pose_marker.color.b = 0.0
                self.region_pose_marker.pose.position.z = 0.01
                self.region_pose_marker_label.pose.position.z = 1.5
                self.region_pose_marker.scale.x = 0.5
                self.region_pose_marker.scale.y = 0.5
            else:
                pose_marker.position.x = region[region.keys()[i]]['pose']['position'][0]
                pose_marker.position.y = region[region.keys()[i]]['pose']['position'][1]
                pose_text.position.x = region[region.keys()[i]]['pose']['position'][0]
                pose_text.position.y = region[region.keys()[i]]['pose']['position'][1]
                self.region_pose_marker_label.text = region.keys()[i]
                self.region_pose_marker.color.r = 0.5
                self.region_pose_marker.color.g = 0.0
                self.region_pose_marker.color.b = 0.0
                self.region_pose_marker_label.pose.position.z = 0.5

                self.region_pose_marker.scale.x = 1.0
                self.region_pose_marker.scale.y = 1.0

            self.region_pose_marker.header.frame_id = 'tiago1/map'

            self.region_pose_marker.type = self.region_pose_marker.CYLINDER
            self.region_pose_marker.id = self.marker_id_counter
            self.region_pose_marker.action = self.region_pose_marker.ADD
            self.region_pose_marker.scale.z = 0.01
            self.region_pose_marker.color.a = 1.0

            self.region_pose_marker_label.header.frame_id = 'tiago1/map'

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



