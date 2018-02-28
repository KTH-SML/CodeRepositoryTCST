#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import yaml
import codecs
from math import atan2, cos, sin, pi, atan
from pyquaternion import Quaternion

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QDialog, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QVBoxLayout, QGridLayout, QRadioButton, QGroupBox, QCheckBox, QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QRectF, QSizeF, QLineF, Slot, pyqtSlot, Qt
from python_qt_binding.QtGui import QImageReader, QImage, QPixmap, QMouseEvent, QPen, QBrush, QColor, QFont

class Map_dialog(QDialog):
    signalCheckBoxIndex = pyqtSignal([int], [int])
    def __init__(self, scenario, current_graphicsScene):

        super(Map_dialog, self).__init__()
        self.setObjectName('Map_dialog')
        self.scenario = scenario

        checkBoxChanged = pyqtSignal([int], [int])

        #main_widget = SimulationWidget()
        print(self.scenario)

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'map.ui')
        loadUi(ui_file, self)

        self.grid = QGridLayout()
        self.grid.addWidget(self.groupBox, 0, 0)
        self.groupBox_list = []
        self.FTS_matrix = []
        self.vbox_list = []
        self.vbox = QVBoxLayout()

        self.clicked = False
        self.arrow_list = []

        #self.grid.addWidget(self.graphicsView, 0, 0, 2, 1)
        #self.grid.addWidget(self.button_save_ROI, 0, 2)
        #self.grid.addWidget(self.button_reset, 1, 2)
        self.setLayout(self.grid)

        map_yaml = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, 'map.yaml')
        self.loadConfig(map_yaml)

        self.button_save_FTS.pressed.connect(self.on_button_FTS_save_pressed)
        self.button_save_FTS.setEnabled(False)
        self.button_reset.pressed.connect(self.on_button_reset_pressed)
        self.button_set_edges.pressed.connect(self.on_button_set_edges_pressed)
        self.button_cancel.clicked.connect(self.on_button_cancel_pressed)
        #self.button_set_edges.setEnabled(False)

        self.regionCounter = 0
        self.region_list = []
        self.ellipse_items = []
        self.region_of_interest = {}
        self.ellipse_items_labels = []
        self.pixel_coords_list = []
        self.line_dict = {}

        self.graphicsScene = current_graphicsScene
        self.graphicsScene.signalMousePressedPos.connect(self.pointSelection)
        self.graphicsScene.signalMouseReleasedPos.connect(self.pointRelease)
        self.graphicsScene.signalMouseMovePos.connect(self.mouseMove)

        if self.scenario == 'pal_office':
            map = 'map.pgm'
        else:
            map = 'map.png'

        map_file = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.scenario, map)
        print(map_file)
        pixmap = QPixmap(map_file)
        mapSize = pixmap.size()
        #self.graphicsScene.addPixmap(pixmap)

        self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + mapSize.height())

        #self.graphicsScene.addCoordinateSystem(self.worldOrigin, 0.0)
        self.graphicsView.setScene(self.graphicsScene)

    @Slot(bool)
    def on_button_FTS_save_pressed(self):
        self.button_save_FTS.setEnabled(False)
        for i in range(0, len(self.FTS_matrix)):
            edges = []

            for j in range(0, len(self.FTS_matrix[0])):
                if (self.FTS_matrix[i][j].checkState() == 2):
                    edges.append({'cost' : 1.0, 'target': self.region_list[j]})
                    #edge.update({'target': self.region_list[j]})
                    print(edges)

            self.region_of_interest[self.region_list[i]].update({'edges' : edges})

        print('start saving')
        data = dict(
            self.region_of_interest,
            #actions = 'TODO'
        )
        print(data)
        env_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        with codecs.open(env_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(data, outfile, default_flow_style=False)

        self.accept()

    @Slot(bool)
    def on_button_reset_pressed(self):
        print('Reset')
        for i in range(0, self.regionCounter):
            self.graphicsScene.removeItem(self.ellipse_items[i])
            self.graphicsScene.removeItem(self.ellipse_items_labels[i])
            self.grid.removeWidget(self.groupBox_list[0])
            self.groupBox_list[0].deleteLater()
            del self.groupBox_list[0]
        for i in range(0, len(self.line_dict)):
            self.graphicsScene.removeItem(self.line_dict[self.line_dict.keys()[i]])
        for i in range(0, len(self.arrow_list)):
            self.graphicsScene.removeArrow(self.arrow_list[i])
        self.vbox = QVBoxLayout()
        self.vbox_list = []
        self.FTS_matrix = []
        self.ellipse_items = []
        self.ellipse_items_labels = []
        self.region_of_interest = {}
        self.region_list = []
        self.regionCounter = 0
        self.pixel_coords_list = []
        self.button_save_FTS.setEnabled(False)
        self.line_dict = {}
        self.arrow_list = []

    @Slot(bool)
    def on_button_set_edges_pressed(self):
        for i in range(0, len(self.FTS_matrix)):
            for j in range(0, len(self.FTS_matrix[0])):
                self.FTS_matrix[i][j].setCheckState(Qt.Checked)

    def pointSelection(self, pos):
        print('scene')
        print(pos)
        self.clicked = True
        self.current_arrow = []
        self.regionCounter += 1
        self.pixel_coords_list.append(pos)
        position_of_interest = {'position' : self.pixelToWorld(pos)}
        self.pose_of_interest = {'pose': position_of_interest}
        self.region_list.append('r' + str(self.regionCounter).zfill(2))
        #self.region_of_interest.update({'r' + str(self.regionCounter).zfill(2) : position_of_interest})
        print(self.region_of_interest)
        markerSize = 13

        self.ellipse_items.append(QGraphicsEllipseItem(QRectF(QPointF(pos.x() - markerSize/2, pos.y() - markerSize/2), QSizeF(markerSize, markerSize))))
        self.ellipse_items[self.regionCounter - 1].setBrush(QBrush(QColor('red')))
        self.graphicsScene.addItem(self.ellipse_items[self.regionCounter - 1])

        label_font = QFont()
        label_font.setPointSize(15)
        regionString = 'r' + str(self.regionCounter).zfill(2)
        self.ellipse_items_labels.append(QGraphicsTextItem(regionString))
        self.ellipse_items_labels[self.regionCounter - 1].setPos(pos)
        self.ellipse_items_labels[self.regionCounter - 1].setFont(label_font)
        self.graphicsScene.addItem(self.ellipse_items_labels[self.regionCounter - 1])
        self.button_save_FTS.setEnabled(True)

        groupBox = QGroupBox(regionString)
        self.groupBox_list.append(groupBox)
        checkBox_list = []
        self.FTS_matrix.append(checkBox_list)

        vbox = QVBoxLayout()
        self.vbox_list.append(vbox)
        for i in range(0, self.regionCounter):
            if i == (self.regionCounter-1):
                for j in range(0, i+1):
                    self.FTS_matrix[i].append(QCheckBox('r' + str(j+1).zfill(2)))
                    self.FTS_matrix[i][j].stateChanged.connect(self.edge_both_ways)
                    self.vbox_list[i].addWidget(self.FTS_matrix[i][j])
            else:
                self.FTS_matrix[i].append(QCheckBox('r' + str(self.regionCounter).zfill(2)))
                self.FTS_matrix[i][self.regionCounter-1].stateChanged.connect(self.edge_both_ways)
                self.vbox_list[i].addWidget(self.FTS_matrix[i][self.regionCounter-1])

        for i in range(0, len(self.groupBox_list)):
            self.groupBox_list[i].setLayout(self.vbox_list[i])
            self.grid.addWidget(self.groupBox_list[i], 0, i+1, Qt.AlignRight)

    def pointRelease(self, pos):
        self.clicked = False
        print('release')
        print(pos)
        deltay = -pos.y() + self.pixel_coords_list[self.regionCounter -1].y()
        deltax = pos.x() - self.pixel_coords_list[self.regionCounter -1].x()
        theta = atan2(deltay, deltax)
        print(theta)
        quat = Quaternion(axis=(0.0, 0.0, 1.0), radians=theta)
        print(quat)
        self.pose_of_interest['pose'].update({'orientation' : (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))})
        self.region_of_interest['r' + str(self.regionCounter).zfill(2)] = self.pose_of_interest
        self.arrow_list.append(self.current_arrow)

    def mouseMove(self, pos):
        arrow_length = 50
        if self.clicked:
            theta = atan2((pos.y() - self.pixel_coords_list[self.regionCounter - 1].y()) , (pos.x() - self.pixel_coords_list[self.regionCounter - 1].x()))
            end_point = QPointF(self.pixel_coords_list[self.regionCounter - 1].x() + arrow_length * cos(theta), self.pixel_coords_list[self.regionCounter - 1].y() + arrow_length * sin(theta))
            if len(self.current_arrow) > 0:
                self.graphicsScene.removeArrow(self.current_arrow)
            self.current_arrow = self.graphicsScene.addArrow(self.pixel_coords_list[self.regionCounter - 1], end_point)


    def pixelToWorld(self, pixel_coords = QPointF()):
        world_coords = ((pixel_coords.x() - self.worldOrigin.x()) * self.map_resolution, -(pixel_coords.y() - self.worldOrigin.y()) * self.map_resolution, 0.0)
        return world_coords

    @Slot(bool)
    def edge_both_ways(self, state):        
        for i in range(0, self.regionCounter):
            for j in range(0, self.regionCounter):
                if self.FTS_matrix[i][j].checkState() != self.FTS_matrix[j][i].checkState():
                    if state == 2:
                        self.FTS_matrix[j][i].setCheckState(2)
                        if str((j+1)*(i+1)) not in self.line_dict.keys():
                            self.line_dict[str((j+1)*(i+1))] = QGraphicsLineItem(QLineF(self.pixel_coords_list[i], self.pixel_coords_list[j]))
                            self.graphicsScene.addItem(self.line_dict[str((j+1)*(i+1))])
                            print((j+1)*(i+1))
                    elif state == 0:
                        print((j+1)*(i+1))
                        self.FTS_matrix[j][i].setCheckState(0)
                        if str((j+1)*(i+1)) in self.line_dict.keys():
                            self.graphicsScene.removeItem(self.line_dict[str((j+1)*(i+1))])
                            del self.line_dict[str((j+1)*(i+1))]

    @Slot(bool)
    def on_button_cancel_pressed(self):
        self.on_button_reset_pressed()
        self.accept()

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



