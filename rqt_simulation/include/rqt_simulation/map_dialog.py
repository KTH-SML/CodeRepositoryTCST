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
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QDialog, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QVBoxLayout, QGridLayout, QRadioButton, QGroupBox, QCheckBox, QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem, QFileDialog
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QRectF, QSizeF, QLineF, Slot, pyqtSlot, Qt
from python_qt_binding.QtGui import QImageReader, QImage, QPixmap, QMouseEvent, QPen, QBrush, QColor, QFont

class Map_dialog(QDialog):
    #signalCheckBoxIndex = pyqtSignal([int], [int])
    def __init__(self, current_graphicsScene, FTS):
        super(Map_dialog, self).__init__()
        self.setObjectName('Map_dialog')

        #checkBoxChanged = pyqtSignal([int], [int])

        self.graphicsScene = current_graphicsScene
        self.FTS = FTS

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'map.ui')
        loadUi(ui_file, self)

        self.grid = QGridLayout()

        self.groupBox_list = []
        self.FTS_matrix = []
        self.vbox_list = []
        self.vbox = QVBoxLayout()

        sorted_keys = self.FTS.region_of_interest.keys()
        sorted_keys.sort()

        for i in range(0, self.graphicsScene.regionCounter):
            groupBox = QGroupBox(sorted_keys[i])
            self.groupBox_list.append(groupBox)
            checkBox_list = []
            self.FTS_matrix.append(checkBox_list)

            vbox = QVBoxLayout()
            self.vbox_list.append(vbox)
            for j in range(0, self.graphicsScene.regionCounter):
                self.FTS_matrix[i].append(QCheckBox('r' + str(j+1).zfill(2)))
                self.FTS_matrix[i][j].stateChanged.connect(self.edge_both_ways)
                self.vbox_list[i].addWidget(self.FTS_matrix[i][j])

            self.groupBox_list[i].setLayout(self.vbox_list[i])
            self.grid.addWidget(self.groupBox_list[i], 0, i+1, Qt.AlignRight)

        for i in range(0, self.graphicsScene.regionCounter):
            for j in range(0, self.graphicsScene.regionCounter):
                if (str(i+1) + '-' + str(j+1)) in self.graphicsScene.line_dict.keys():
                    self.FTS_matrix[i][j].setCheckState(2)

        self.clicked = False

        self.scrollAreaWidgetContents.setLayout(self.grid)

        self.button_save_FTS.pressed.connect(self.on_button_FTS_save_pressed)
        #self.button_save_FTS.setEnabled(False)
        self.button_reset.pressed.connect(self.on_button_reset_pressed)
        self.button_set_edges.pressed.connect(self.on_button_set_edges_pressed)
        self.button_cancel.clicked.connect(self.on_button_cancel_pressed)
        self.button_ROI.clicked.connect(self.remove_last_ROI)
        #self.button_ROI.setEnabled(False)
        self.button_delete_edges.clicked.connect(self.delete_edges)
        self.button_load_FTS.clicked.connect(self.load_FTS)
        #self.button_set_edges.setEnabled(False)

        self.graphicsScene.signalMousePressedPos.connect(self.pointSelection)
        self.graphicsScene.signalMouseReleasedPos.connect(self.pointRelease)
        self.graphicsScene.signalMouseMovePos.connect(self.mouseMove)

        self.graphicsView.setScene(self.graphicsScene)

    @Slot(bool)
    def on_button_FTS_save_pressed(self):
        self.button_save_FTS.setEnabled(False)
        for i in range(0, len(self.FTS_matrix)):
            for j in range(0, len(self.FTS_matrix[0])):
                if (self.FTS_matrix[i][j].checkState() == 2):
                    self.FTS.add_edge(self.FTS.region_of_interest.keys()[i], self.FTS.region_of_interest.keys()[j], cost=1.0)

        print('start saving')
        data = {'FTS' : self.FTS.region_of_interest}
        print(data)
        env_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        with codecs.open(env_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(data, outfile, default_flow_style=False)

        self.accept()

    @Slot(bool)
    def on_button_reset_pressed(self):
        print('Reset')

        for i in range(0, self.graphicsScene.regionCounter):
            self.grid.removeWidget(self.groupBox_list[0])
            self.groupBox_list[0].deleteLater()
            del self.groupBox_list[0]

        self.graphicsScene.reset()
        self.vbox = QVBoxLayout()
        self.vbox_list = []
        self.FTS_matrix = []
        self.FTS.region_of_interest = {}
        self.button_save_FTS.setEnabled(False)

    @Slot(bool)
    def remove_last_ROI(self):
        del self.FTS_matrix[self.graphicsScene.regionCounter-1]
        del self.FTS.region_of_interest['r' + str(self.graphicsScene.regionCounter).zfill(2)]
        del self.vbox_list[self.graphicsScene.regionCounter-1]

        self.grid.removeWidget(self.groupBox_list[self.graphicsScene.regionCounter-1])
        self.groupBox_list[self.graphicsScene.regionCounter-1].deleteLater()
        del self.groupBox_list[self.graphicsScene.regionCounter-1]

        for i in range(0, self.graphicsScene.regionCounter-1):
            self.vbox_list[i].removeWidget(self.FTS_matrix[i][self.graphicsScene.regionCounter-1])
            self.FTS_matrix[i][self.graphicsScene.regionCounter-1].deleteLater()
            del self.FTS_matrix[i][self.graphicsScene.regionCounter-1]
            if self.graphicsScene.regionCounter < (i+1):
                if ((str(self.graphicsScene.regionCounter) + '-' + str(i+1)) in self.graphicsScene.line_dict.keys()):
                    self.graphicsScene.remove_edge((str(self.graphicsScene.regionCounter) + '-' + str(i+1)))
            else:
                if ((str(i+1) + '-' + str(self.graphicsScene.regionCounter)) in self.graphicsScene.line_dict.keys()):
                    self.graphicsScene.remove_edge((str(i+1) + '-' + str(self.graphicsScene.regionCounter)))

        self.graphicsScene.remove_ROI()

        if self.graphicsScene.regionCounter < 1:
            self.button_ROI.setEnabled(False)

    @Slot(bool)
    def on_button_set_edges_pressed(self):
        for i in range(0, len(self.FTS_matrix)):
            for j in range(0, len(self.FTS_matrix[0])):
                self.FTS_matrix[i][j].setCheckState(Qt.Checked)

    @Slot(bool)
    def delete_edges(self):
        for i in range(0, len(self.FTS_matrix)):
            for j in range(0, len(self.FTS_matrix[0])):
                self.FTS_matrix[i][j].setCheckState(Qt.Unchecked)


    def pointSelection(self, pos):
        print('scene')
        print(pos)
        self.clicked = True
        self.current_arrow = []
        self.graphicsScene.add_ROI(pos)
        print(self.graphicsScene.regionCounter)
        self.pose_of_interest = {'position' : self.graphicsScene.pixelToWorld(pos)}
        print(self.graphicsScene.pixelToWorld(pos))

        regionString = 'r' + str(self.graphicsScene.regionCounter).zfill(2)

        self.button_save_FTS.setEnabled(True)

        groupBox = QGroupBox(regionString)
        self.groupBox_list.append(groupBox)
        checkBox_list = []
        self.FTS_matrix.append(checkBox_list)

        vbox = QVBoxLayout()
        self.vbox_list.append(vbox)
        for i in range(0, self.graphicsScene.regionCounter):
            if i == (self.graphicsScene.regionCounter-1):
                for j in range(0, i+1):
                    self.FTS_matrix[i].append(QCheckBox('r' + str(j+1).zfill(2)))
                    self.FTS_matrix[i][j].stateChanged.connect(self.edge_both_ways)
                    self.vbox_list[i].addWidget(self.FTS_matrix[i][j])
            else:
                self.FTS_matrix[i].append(QCheckBox('r' + str(self.graphicsScene.regionCounter).zfill(2)))
                self.FTS_matrix[i][self.graphicsScene.regionCounter-1].stateChanged.connect(self.edge_both_ways)
                self.vbox_list[i].addWidget(self.FTS_matrix[i][self.graphicsScene.regionCounter-1])

        for i in range(0, len(self.groupBox_list)):
            self.groupBox_list[i].setLayout(self.vbox_list[i])
            self.grid.addWidget(self.groupBox_list[i], 0, i+1, Qt.AlignRight)

    def pointRelease(self, pos):
        self.clicked = False
        pixel_coords = self.graphicsScene.items_dict['r' + str(self.graphicsScene.regionCounter).zfill(2)]['pixel_coords']
        deltay = -pos.y() + pixel_coords.y()
        deltax = pos.x() - pixel_coords.x()
        theta = atan2(deltay, deltax)
        quat = Quaternion(axis=(0.0, 0.0, 1.0), radians=theta)
        self.pose_of_interest.update({'orientation' : (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))})
        edges = []
        self.FTS.add_region('r' + str(self.graphicsScene.regionCounter).zfill(2), edges, pose = self.pose_of_interest)
        self.graphicsScene.items_dict['r' + str(self.graphicsScene.regionCounter).zfill(2)].update({'arrow' : self.current_arrow})
        self.button_ROI.setEnabled(True)

    def mouseMove(self, pos):
        arrow_length = 50
        if self.clicked:
            pixel_coords = self.graphicsScene.items_dict['r' + str(self.graphicsScene.regionCounter).zfill(2)]['pixel_coords']
            theta = atan2((pos.y() - pixel_coords.y()) , (pos.x() - pixel_coords.x()))
            end_point = QPointF(pixel_coords.x() + arrow_length * cos(theta), pixel_coords.y() + arrow_length * sin(theta))
            if len(self.current_arrow) > 0:
                self.graphicsScene.removeArrow(self.current_arrow)
            self.current_arrow = self.graphicsScene.addArrow(pixel_coords, end_point)

    @Slot(bool)
    def edge_both_ways(self, state):        
        for i in range(0, self.graphicsScene.regionCounter):
            for j in range(0, self.graphicsScene.regionCounter):
                if self.FTS_matrix[i][j].checkState() != self.FTS_matrix[j][i].checkState():
                    if state == 2:
                        self.FTS_matrix[j][i].setCheckState(2)
                        if i < j:
                            if (str(i+1) + '-' + str(j+1)) not in self.graphicsScene.line_dict.keys():
                                self.graphicsScene.add_edge(i+1, j+1)
                                print((str(i+1) + '-' + str(j+1)))
                        else:
                            if (str(j+1) + '-' + str(i+1)) not in self.graphicsScene.line_dict.keys():
                                self.graphicsScene.add_edge(j+1, i+1)
                                print((str(j+1) + '-' + str(i+1)))

                    elif state == 0:
                        print((str(i+1) + '-' + str(j+1)))
                        self.FTS_matrix[j][i].setCheckState(0)
                        if i < j:
                            if (str(i+1) + '-' + str(j+1)) in self.graphicsScene.line_dict.keys():
                                self.graphicsScene.remove_edge(str(i+1) + '-' + str(j+1))
                        else:
                            if (str(j+1) + '-' + str(i+1)) in self.graphicsScene.line_dict.keys():
                                self.graphicsScene.remove_edge(str(j+1) + '-' + str(i+1))

    @Slot(bool)
    def on_button_cancel_pressed(self):
        self.on_button_reset_pressed()
        self.accept()

    @Slot(bool)
    def load_FTS(self):
        directory = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS')
        File_dialog = QFileDialog(directory=directory, filter='.yaml')
        FTS_file = File_dialog.getOpenFileName()
        print(FTS_file[0])
        stream = file(FTS_file[0], 'r')
        data = yaml.load(stream)
        self.FTS.region_of_interest = data['FTS']
        sorted_keys = self.FTS.region_of_interest.keys()
        sorted_keys.sort()
        stream.close()

        self.graphicsScene.reset()

        arrow_length = 50

        for i in range(0, len(self.FTS.region_of_interest)):
            region_string = 'r' + str(i+1).zfill(2)
            pixel_coords = self.graphicsScene.worldToPixel(self.FTS.region_of_interest[sorted_keys[i]]['pose']['position'])
            self.graphicsScene.add_ROI(pixel_coords)

            quaternion = Quaternion(self.FTS.region_of_interest[sorted_keys[i]]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]
            end_point = QPointF(pixel_coords.x() + arrow_length * cos(theta), pixel_coords.y() - arrow_length * sin(theta))
            arrow = self.graphicsScene.addArrow(pixel_coords, end_point)
            self.graphicsScene.items_dict[region_string]['arrow'] = arrow

            groupBox = QGroupBox(sorted_keys[i])
            self.groupBox_list.append(groupBox)
            checkBox_list = []
            self.FTS_matrix.append(checkBox_list)

            vbox = QVBoxLayout()
            self.vbox_list.append(vbox)

            for j in range(0, len(self.FTS.region_of_interest)):
                    self.FTS_matrix[i].append(QCheckBox(sorted_keys[j]))
                    self.FTS_matrix[i][j].stateChanged.connect(self.edge_both_ways)
                    self.vbox_list[i].addWidget(self.FTS_matrix[i][j])

            self.groupBox_list[i].setLayout(self.vbox_list[i])
            self.grid.addWidget(self.groupBox_list[i], 0, i+1, Qt.AlignRight)

        for i in range(0, len(self.FTS.region_of_interest)):
            for j in range(0, len(self.FTS.region_of_interest[sorted_keys[i]]['edges'])):
                index = sorted_keys.index(self.FTS.region_of_interest[sorted_keys[i]]['edges'][j]['target'])
                self.FTS_matrix[i][index].setCheckState(2)
