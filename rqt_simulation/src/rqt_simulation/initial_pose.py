#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import yaml
import codecs
from math import atan2, cos, sin, pi, atan

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QDialog, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QVBoxLayout, QGridLayout, QRadioButton, QGroupBox, QCheckBox
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QRectF, QSizeF, QLineF, Slot, pyqtSlot, Qt
from python_qt_binding.QtGui import QImageReader, QImage, QPixmap, QMouseEvent, QPen, QBrush, QColor, QFont, QGraphicsEllipseItem, QGraphicsTextItem

class Initial_pose(QDialog):
    def __init__(self, scenario, num_robots, current_graphicsScene):

        super(Initial_pose, self).__init__()
        self.setObjectName('Initial_pose')
        self.scenario = scenario
        self.num_robots = num_robots

        print(self.scenario)

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'init_pose.ui')
        loadUi(ui_file, self)

        map_yaml = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, 'map.yaml')
        self.loadConfig(map_yaml)

        self.button_save.pressed.connect(self.on_button_save_pressed)
        self.button_save.setEnabled(False)
        self.button_reset.pressed.connect(self.on_button_reset_pressed)
        self.button_cancel.pressed.connect(self.on_button_cancel_pressed)
        self.textBrowser_robot_1.hide()
        self.textBrowser_robot_2.hide()
        #self.button_set_edges.setEnabled(False)

        self.regionCounter = 0
        self.region_list = []
        self.ellipse_items = []
        self.region_of_interest = {}
        self.ellipse_items_labels = []

        self.graphicsScene = current_graphicsScene
        self.graphicsScene.signalMousePos.connect(self.pointSelection)

        if self.scenario == 'pal_office':
            map = 'map.pgm'
        else:
            map = 'map.png'

        map_file = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, map)
        print(map_file)
        pixmap = QPixmap(map_file)
        mapSize = pixmap.size()
        self.graphicsScene.addPixmap(pixmap)

        self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + mapSize.height())

        self.graphicsScene.addCoordinateSystem(self.worldOrigin, 0.0)
        self.graphicsView.setScene(self.graphicsScene)

    @Slot(bool)
    def on_button_save_pressed(self):
        self.accept()

    @Slot(bool)
    def on_button_cancel_pressed(self):
        self.on_button_reset_pressed()
        self.accept()

    @Slot(bool)
    def on_button_reset_pressed(self):
        print('Reset')
        for i in range(0, self.regionCounter):
            self.graphicsScene.removeItem(self.ellipse_items[i])
            self.graphicsScene.removeItem(self.ellipse_items_labels[i])
        self.ellipse_items = []
        self.ellipse_items_labels = []
        self.region_of_interest = {}
        self.region_list = []
        self.regionCounter = 0
        self.button_save.setEnabled(False)
        self.textBrowser_robot_1.hide()
        self.textBrowser_robot_2.hide()

    def pointSelection(self, pos):
        if self.num_robots > self.regionCounter:
            print('scene')
            print(pos)
            self.regionCounter += 1
            self.region_of_interest.update({'start_' + str(self.regionCounter).zfill(2) : self.pixelToWorld(pos)})
            print(self.region_of_interest)
            markerSize = 11

            self.ellipse_items.append(QGraphicsEllipseItem(QRectF(QPointF(pos.x() - markerSize/2, pos.y() - markerSize/2), QSizeF(markerSize, markerSize))))
            self.ellipse_items[self.regionCounter - 1].setBrush(QBrush(QColor('green')))
            self.graphicsScene.addItem(self.ellipse_items[self.regionCounter - 1])

            regionString = 'start_' + str(self.regionCounter).zfill(2)
            self.ellipse_items_labels.append(QGraphicsTextItem(regionString))
            self.ellipse_items_labels[self.regionCounter - 1].setPos(pos)
            self.graphicsScene.addItem(self.ellipse_items_labels[self.regionCounter - 1])
            self.button_save.setEnabled(True)

            if self.regionCounter == 1:
                self.textBrowser_robot_1.setText('The selected initial position for robot_1 is: ' + str(self.pixelToWorld(pos)))
                self.textBrowser_robot_1.show()

            if self.regionCounter == 2:
                self.textBrowser_robot_2.setText('The selected initial position for robot_2 is: ' + str(self.pixelToWorld(pos)))
                self.textBrowser_robot_2.show()

    def pixelToWorld(self, pixel_coords = QPointF()):
        world_coords = (round((pixel_coords.x() - self.worldOrigin.x()) * self.map_resolution, 3), round(-(pixel_coords.y() - self.worldOrigin.y()) * self.map_resolution, 3), 0.0)
        return world_coords


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
