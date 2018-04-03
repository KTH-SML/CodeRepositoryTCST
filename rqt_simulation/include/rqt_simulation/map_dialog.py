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
    signalCheckBoxIndex = pyqtSignal([int], [int])
    def __init__(self, current_graphicsScene, FTS):

        super(Map_dialog, self).__init__()
        self.setObjectName('Map_dialog')
        #self.scenario = scenario

        checkBoxChanged = pyqtSignal([int], [int])

        #main_widget = SimulationWidget()
        #print(self.scenario)
        self.graphicsScene = current_graphicsScene
        self.FTS = FTS

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'map.ui')
        loadUi(ui_file, self)

        self.grid = QGridLayout()
        #self.grid.setColumnMinimumWidth(0, 50)
        #self.grid.addWidget(self.groupBox_edges, 0, 0)

        self.groupBox_list = []
        self.FTS_matrix = []
        self.vbox_list = []
        self.vbox = QVBoxLayout()

        for i in range(0, self.graphicsScene.regionCounter):
            groupBox = QGroupBox(self.FTS.region_of_interest.keys()[i])
            self.groupBox_list.append(groupBox)
            #self.groupBox_list[self.graphicsScene.regionCounter-1].setMinimumWidth(50)
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
                if (str(i) + '-' + str(j)) in self.graphicsScene.line_dict.keys():
                    self.FTS_matrix[i][j].setCheckState(2)

        self.clicked = False
        #self.arrow_list = []

        #self.grid.addWidget(self.graphicsView, 0, 0, 2, 1)
        #self.grid.addWidget(self.button_save_ROI, 0, 2)
        #self.grid.addWidget(self.button_reset, 1, 2)
        #self.groupBox_edges.setLayout(self.grid)
        self.scrollAreaWidgetContents.setLayout(self.grid)

        #map_yaml = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', self.graphicsScene.scenario, 'map.yaml')
        #self.loadConfig(map_yaml)

        self.button_save_FTS.pressed.connect(self.on_button_FTS_save_pressed)
        self.button_save_FTS.setEnabled(False)
        self.button_reset.pressed.connect(self.on_button_reset_pressed)
        self.button_set_edges.pressed.connect(self.on_button_set_edges_pressed)
        self.button_cancel.clicked.connect(self.on_button_cancel_pressed)
        self.button_ROI.clicked.connect(self.remove_last_ROI)
        self.button_ROI.setEnabled(False)
        self.button_delete_edges.clicked.connect(self.delete_edges)
        self.button_load_FTS.clicked.connect(self.load_FTS)
        #self.button_set_edges.setEnabled(False)

        #self.graphicsScene.regionCounter = 0
        #self.region_list = []
        #self.ellipse_items = []
        #self.region_of_interest = {}
        #self.ellipse_items_labels = []
        #self.pixel_coords_list = []
        #self.line_dict = {}


        self.graphicsScene.signalMousePressedPos.connect(self.pointSelection)
        self.graphicsScene.signalMouseReleasedPos.connect(self.pointRelease)
        self.graphicsScene.signalMouseMovePos.connect(self.mouseMove)

        self.graphicsView.setScene(self.graphicsScene)

    @Slot(bool)
    def on_button_FTS_save_pressed(self):
        self.button_save_FTS.setEnabled(False)
        for i in range(0, len(self.FTS_matrix)):
            #edges = []

            for j in range(0, len(self.FTS_matrix[0])):
                if (self.FTS_matrix[i][j].checkState() == 2):
                    self.FTS.add_edge(self.FTS.region_of_interest.keys()[i], self.FTS.region_of_interest.keys()[j], cost=1.0)
                    #edges.append({'cost' : 1.0, 'target': self.FTS.region_of_interest.keys()[j]})
                    #edge.update({'target': self.region_list[j]})
                    #print('edges')

            #self.FTS.region_of_interest[self.FTS.region_list[i]].update({'edges' : edges})
            #self.FTS.region_of_interest[self.FTS.region_of_interest.keys()[i]]['edges'] = edges

        print('start saving')
        data = dict(
            self.FTS.region_of_interest,
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

        for i in range(0, self.graphicsScene.regionCounter):
            #self.graphicsScene.remove_ROI()
            #self.graphicsScene.removeItem(self.ellipse_items[i])
            #self.graphicsScene.removeItem(self.ellipse_items_labels[i])
            self.grid.removeWidget(self.groupBox_list[0])
            self.groupBox_list[0].deleteLater()
            del self.groupBox_list[0]
       # for i in range(0, len(self.graphicsScene.line_dict)):
            #print(self.graphicsScene.line_dict.keys()[i])
            #print(self.graphicsScene.line_dict[self.graphicsScene.line_dict.keys()[i]])
           # self.graphicsScene.remove_edge(self.graphicsScene.line_dict.keys()[0])
        #for i in range(0, len(self.arrow_list)):
            #self.graphicsScene.removeArrow(self.arrow_list[i])
        self.graphicsScene.reset()
        self.vbox = QVBoxLayout()
        self.vbox_list = []
        self.FTS_matrix = []
        #self.ellipse_items = []
        #self.ellipse_items_labels = []
        self.FTS.region_of_interest = {}
        #self.FTS.region_list = []
        #self.graphicsScene.regionCounter = 0
        #self.pixel_coords_list = []
        self.button_save_FTS.setEnabled(False)
        #self.line_dict = {}
        #self.arrow_list = []

    @Slot(bool)
    def remove_last_ROI(self):



        #self.graphicsScene.removeItem(self.ellipse_items[self.graphicsScene.regionCounter-1])
        #self.graphicsScene.removeItem(self.ellipse_items_labels[self.graphicsScene.regionCounter-1])
        #self.graphicsScene.removeArrow(self.arrow_list[self.graphicsScene.regionCounter-1])
        #del self.arrow_list[self.graphicsScene.regionCounter-1]
        del self.FTS_matrix[self.graphicsScene.regionCounter-1]
        #del self.ellipse_items[self.graphicsScene.regionCounter-1]
        #del self.ellipse_items_labels[self.graphicsScene.regionCounter-1]
        del self.FTS.region_of_interest['r' + str(self.graphicsScene.regionCounter).zfill(2)]
        #del self.FTS.region_list[self.graphicsScene.regionCounter-1]
        #del self.pixel_coords_list[self.graphicsScene.regionCounter-1]
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
            #if ((str(self.graphicsScene.regionCounter) + '-' + str(i+1)) in self.line_dict.keys()) or ((str(i+1) + '-' + str(self.graphicsScene.regionCounter)) in self.line_dict.keys()):

                #self.graphicsScene.removeItem(self.line_dict[str((self.graphicsScene.regionCounter)*(i+1))])
                #del self.line_dict[str((self.graphicsScene.regionCounter)*(i+1))]
        self.graphicsScene.remove_ROI()

        #self.graphicsScene.regionCounter = self.graphicsScene.regionCounter - 1
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
        #self.graphicsScene.regionCounter += 1
        self.graphicsScene.add_ROI(pos)
        print(self.graphicsScene.regionCounter)
        #self.pixel_coords_list.append(pos)
        self.pose_of_interest = {'position' : self.graphicsScene.pixelToWorld(pos)}
        print(self.graphicsScene.pixelToWorld(pos))
        #self.pose_of_interest = position_of_interest
        #self.FTS.region_list.append('r' + str(self.graphicsScene.regionCounter).zfill(2))
        #self.region_of_interest.update({'r' + str(self.graphicsScene.regionCounter).zfill(2) : position_of_interest})

        #markerSize = 13

        #self.ellipse_items.append(QGraphicsEllipseItem(QRectF(QPointF(pos.x() - markerSize/2, pos.y() - markerSize/2), QSizeF(markerSize, markerSize))))
        #self.ellipse_items[self.graphicsScene.regionCounter - 1].setBrush(QBrush(QColor('red')))
        #self.graphicsScene.addItem(self.ellipse_items[self.graphicsScene.regionCounter - 1])

        #label_font = QFont()
        #label_font.setPointSize(15)
        regionString = 'r' + str(self.graphicsScene.regionCounter).zfill(2)
        #self.ellipse_items_labels.append(QGraphicsTextItem(regionString))
        #self.ellipse_items_labels[self.graphicsScene.regionCounter - 1].setPos(pos)
        #self.ellipse_items_labels[self.graphicsScene.regionCounter - 1].setFont(label_font)
        #self.graphicsScene.addItem(self.ellipse_items_labels[self.graphicsScene.regionCounter - 1])


        self.button_save_FTS.setEnabled(True)

        groupBox = QGroupBox(regionString)
        self.groupBox_list.append(groupBox)
        #self.groupBox_list[self.graphicsScene.regionCounter-1].setMinimumWidth(50)
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
        print('release')
        print(pos)
        deltay = -pos.y() + self.graphicsScene.pixel_coords_list[self.graphicsScene.regionCounter -1].y()
        deltax = pos.x() - self.graphicsScene.pixel_coords_list[self.graphicsScene.regionCounter -1].x()
        theta = atan2(deltay, deltax)
        print('theta selected')
        print(theta)
        quat = Quaternion(axis=(0.0, 0.0, 1.0), radians=theta)
        print(quat)
        self.pose_of_interest.update({'orientation' : (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))})
        #self.FTS.region_of_interest['r' + str(self.graphicsScene.regionCounter).zfill(2)] = self.pose_of_interest
        edges = []
        self.FTS.add_region('r' + str(self.graphicsScene.regionCounter).zfill(2), edges, pose = self.pose_of_interest)
        print(self.FTS)
        self.graphicsScene.arrow_list.append(self.current_arrow)
        self.button_ROI.setEnabled(True)
        print(self.FTS.region_of_interest)

    def mouseMove(self, pos):
        arrow_length = 50
        if self.clicked:
            theta = atan2((pos.y() - self.graphicsScene.pixel_coords_list[self.graphicsScene.regionCounter - 1].y()) , (pos.x() - self.graphicsScene.pixel_coords_list[self.graphicsScene.regionCounter - 1].x()))
            end_point = QPointF(self.graphicsScene.pixel_coords_list[self.graphicsScene.regionCounter - 1].x() + arrow_length * cos(theta), self.graphicsScene.pixel_coords_list[self.graphicsScene.regionCounter - 1].y() + arrow_length * sin(theta))
            if len(self.current_arrow) > 0:
                self.graphicsScene.removeArrow(self.current_arrow)
            self.current_arrow = self.graphicsScene.addArrow(self.graphicsScene.pixel_coords_list[self.graphicsScene.regionCounter - 1], end_point)

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
                                #self.FTS.add_edge('r' + str(i+1).zfill(2), 'r' + str(j+1).zfill(2), cost=1.0)
                                #self.FTS.add_edge('r' + str(j+1).zfill(2), 'r' + str(i+1).zfill(2), cost=1.0)
                                #self.line_dict[(str(j+1) + '-' + str(i+1))] = QGraphicsLineItem(QLineF(self.graphicsScene.pixel_coords_list[i], self.graphicsScene.pixel_coords_list[j]))
                                #self.graphicsScene.addItem(self.line_dict[(str(j+1) + '-' + str(i+1))])
                                print((str(i+1) + '-' + str(j+1)))
                        else:
                            if (str(j+1) + '-' + str(i+1)) not in self.graphicsScene.line_dict.keys():
                                self.graphicsScene.add_edge(j+1, i+1)
                                #self.FTS.add_edge('r' + str(i+1).zfill(2), 'r' + str(j+1).zfill(2), cost=1.0)
                                #self.FTS.add_edge('r' + str(j+1).zfill(2), 'r' + str(i+1).zfill(2), cost=1.0)
                                #self.line_dict[(str(j+1) + '-' + str(i+1))] = QGraphicsLineItem(QLineF(self.graphicsScene.pixel_coords_list[i], self.graphicsScene.pixel_coords_list[j]))
                                #self.graphicsScene.addItem(self.line_dict[(str(j+1) + '-' + str(i+1))])
                                print((str(j+1) + '-' + str(i+1)))

                    elif state == 0:
                        print((str(i+1) + '-' + str(j+1)))
                        self.FTS_matrix[j][i].setCheckState(0)
                        if (str(i+1) + '-' + str(j+1)) in self.graphicsScene.line_dict.keys():
                            self.graphicsScene.remove_edge(str(i+1) + '-' + str(j+1))
                            #self.FTS.remove_edge('r' + str(i+1).zfill(2), 'r' + str(j+1).zfill(2))
                            #self.FTS.remove_edge('r' + str(j+1).zfill(2), 'r' + str(i+1).zfill(2))
                            #self.graphicsScene.removeItem(self.line_dict[(str(j+1) + '-' + str(i+1))])

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
        self.FTS.region_of_interest = yaml.load(stream)
        sorted_keys = self.FTS.region_of_interest.keys()
        sorted_keys.sort()
        stream.close()

        self.graphicsScene.reset()

        arrow_length = 50

        for i in range(0, len(self.FTS.region_of_interest)):
            pixel_coords = self.graphicsScene.worldToPixel(self.FTS.region_of_interest[sorted_keys[i]]['pose']['position'])
            self.graphicsScene.add_ROI(pixel_coords)

            quaternion = Quaternion(self.FTS.region_of_interest[sorted_keys[i]]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]
            end_point = QPointF(pixel_coords.x() + arrow_length * cos(theta), pixel_coords.y() - arrow_length * sin(theta))
            arrow = self.graphicsScene.addArrow(pixel_coords, end_point)
            self.graphicsScene.arrow_list.append(arrow)

            groupBox = QGroupBox(sorted_keys[i])
            self.groupBox_list.append(groupBox)
            #self.groupBox_list[self.graphicsScene.regionCounter-1].setMinimumWidth(50)
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

        #self.graphicsScene.regionCounter = len(sorted_keys)
