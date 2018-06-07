# -*- coding: utf-8 -*-

import os
import sys
from math import atan2, cos, sin, pi, atan
from pyquaternion import Quaternion

from python_qt_binding.QtWidgets import QVBoxLayout, QGridLayout, QGroupBox
from python_qt_binding.QtCore import Qt, pyqtSlot, pyqtSignal, QObject, QPointF

from CustomCheckBox import CustomCheckBox
from rqt_simulation_msgs.msg import Sense, Edge, Roi
from geometry_msgs.msg import Pose
from std_msgs.msg import String

class MapUtiles(QObject):
    signalEdgeChanged = pyqtSignal(bool)
    signalNewRoi = pyqtSignal(bool)
    def __init__(self, graphicsScene, FTS):
        super(MapUtiles, self).__init__()

        self.graphicsScene = graphicsScene
        self.graphicsScene.signalMousePressedPos.connect(self.pointSelection)
        self.graphicsScene.signalMouseReleasedPos.connect(self.pointRelease)
        self.graphicsScene.signalMouseMovePos.connect(self.mouseMove)

        self.FTS = FTS

        # Variable is True is left mouse button is holded
        self.clicked = False

        # Set dialog layout to grid
        self.grid = QGridLayout()

        # Initialize variables for FTS matrix
        self.groupBox_list = []
        self.edge_matrix = []
        self.vbox_list = []
        self.vbox = QVBoxLayout()

    # Add ROI with mouse click
    def pointSelection(self, pos):
        print('scene')
        print(pos)
        self.clicked = True
        self.current_arrow = []
        self.graphicsScene.add_ROI(pos)
        print(self.graphicsScene.regionCounter)
        self.pose_of_interest = {'position' : self.graphicsScene.pixelToWorld(pos)}
        print(self.graphicsScene.pixelToWorld(pos))

        self.add_FTS_matrix()

    # Set orientation of ROI by releasing mouse button
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

        roi_msg = self.build_roi_msg('r' + str(self.graphicsScene.regionCounter).zfill(2))
        self.sense_msg = Sense()
        self.sense_msg.rois.append(roi_msg)

        self.signalNewRoi.emit(True)

    # Update orientation arrow while mouse button is pressed
    def mouseMove(self, pos):
        arrow_length = 50
        if self.clicked:
            pixel_coords = self.graphicsScene.items_dict['r' + str(self.graphicsScene.regionCounter).zfill(2)]['pixel_coords']
            theta = atan2((pos.y() - pixel_coords.y()) , (pos.x() - pixel_coords.x()))
            end_point = QPointF(pixel_coords.x() + arrow_length * cos(theta), pixel_coords.y() + arrow_length * sin(theta))
            if len(self.current_arrow) > 0:
                self.graphicsScene.removeArrow(self.current_arrow)
            self.current_arrow = self.graphicsScene.addArrow(pixel_coords, end_point)


    def init_FTS_matrix(self, FTS):

        self.FTS = FTS
        # Sort the ROI's alphabetic, needed for FTS matrix since it's implemented with list
        sorted_keys = self.FTS.region_of_interest.keys()
        sorted_keys.sort()
        # Add all the ROI's and edges
        for i in range(0, len(self.FTS.region_of_interest)):
            groupBox = QGroupBox(sorted_keys[i])
            self.groupBox_list.append(groupBox)
            checkBox_list = []
            self.edge_matrix.append(checkBox_list)

            vbox = QVBoxLayout()
            self.vbox_list.append(vbox)

            for j in range(0, len(self.FTS.region_of_interest)):
                    self.edge_matrix[i].append(CustomCheckBox(sorted_keys[j], i, j))
                    self.edge_matrix[i][j].signalStateChanged.connect(self.edge_both_ways)
                    self.vbox_list[i].addWidget(self.edge_matrix[i][j])

            self.groupBox_list[i].setLayout(self.vbox_list[i])
            self.grid.addWidget(self.groupBox_list[i], 0, i+1, Qt.AlignRight)

        for i in range(0, len(self.FTS.region_of_interest)):
            for j in range(0, len(self.FTS.region_of_interest[sorted_keys[i]]['edges'])):
                index = sorted_keys.index(self.FTS.region_of_interest[sorted_keys[i]]['edges'][j]['target'])
                self.edge_matrix[i][index].setCheckState(2)

    def add_FTS_matrix(self):
        regionString = 'r' + str(self.graphicsScene.regionCounter).zfill(2)

        groupBox = QGroupBox(regionString)
        self.groupBox_list.append(groupBox)
        checkBox_list = []
        self.edge_matrix.append(checkBox_list)

        vbox = QVBoxLayout()
        self.vbox_list.append(vbox)
        for i in range(0, self.graphicsScene.regionCounter):
            if i == (self.graphicsScene.regionCounter-1):
                for j in range(0, i+1):
                    self.edge_matrix[i].append(CustomCheckBox('r' + str(j+1).zfill(2), i, j))
                    self.edge_matrix[i][j].signalStateChanged.connect(self.edge_both_ways)
                    self.vbox_list[i].addWidget(self.edge_matrix[i][j])
            else:
                self.edge_matrix[i].append(CustomCheckBox('r' + str(self.graphicsScene.regionCounter).zfill(2), i, self.graphicsScene.regionCounter-1))
                self.edge_matrix[i][self.graphicsScene.regionCounter-1].signalStateChanged.connect(self.edge_both_ways)
                self.vbox_list[i].addWidget(self.edge_matrix[i][self.graphicsScene.regionCounter-1])

        for i in range(0, len(self.groupBox_list)):
            self.groupBox_list[i].setLayout(self.vbox_list[i])
            self.grid.addWidget(self.groupBox_list[i], 0, i+1, Qt.AlignRight)

    def remove_FTS_matrix(self):
        del self.edge_matrix[self.graphicsScene.regionCounter-1]
        del self.FTS.region_of_interest['r' + str(self.graphicsScene.regionCounter).zfill(2)]
        del self.vbox_list[self.graphicsScene.regionCounter-1]

        self.grid.removeWidget(self.groupBox_list[self.graphicsScene.regionCounter-1])
        self.groupBox_list[self.graphicsScene.regionCounter-1].deleteLater()
        del self.groupBox_list[self.graphicsScene.regionCounter-1]

        for i in range(0, self.graphicsScene.regionCounter-1):
            self.vbox_list[i].removeWidget(self.edge_matrix[i][self.graphicsScene.regionCounter-1])
            self.edge_matrix[i][self.graphicsScene.regionCounter-1].deleteLater()
            del self.edge_matrix[i][self.graphicsScene.regionCounter-1]
            if self.graphicsScene.regionCounter < (i+1):
                if ((str(self.graphicsScene.regionCounter) + '-' + str(i+1)) in self.graphicsScene.line_dict.keys()):
                    self.graphicsScene.remove_edge((str(self.graphicsScene.regionCounter) + '-' + str(i+1)))
            else:
                if ((str(i+1) + '-' + str(self.graphicsScene.regionCounter)) in self.graphicsScene.line_dict.keys()):
                    self.graphicsScene.remove_edge((str(i+1) + '-' + str(self.graphicsScene.regionCounter)))

        self.graphicsScene.remove_ROI()

    def reset_FTS_matrix(self):
        for i in range(0, len(self.edge_matrix)):
            self.remove_FTS_matrix()


    # Set the edges both ways in FTS matrix
    @pyqtSlot(int, int, int)
    def edge_both_ways(self, state, row, col):
        self.sense_msg = Sense()
        start_pose = self.from_dict_to_pose_msg('r' + str(col+1).zfill(2))
        target_pose = self.from_dict_to_pose_msg('r' + str(row+1).zfill(2))
        if state == 2:
            if self.edge_matrix[col][row].checkState() != self.edge_matrix[row][col].checkState():
                self.edge_matrix[col][row].setCheckState(2)
                edge = self.build_edge_msg(start_pose, target_pose, 1.0, True)               
                if edge not in self.sense_msg.edges:
                    self.sense_msg.edges.append(edge)
                edge = self.build_edge_msg(target_pose, start_pose, 1.0, True)
                if edge not in self.sense_msg.edges:
                    self.sense_msg.edges.append(edge)
                if row < col:
                    if (str(row+1) + '-' + str(col+1)) not in self.graphicsScene.line_dict.keys():
                        self.graphicsScene.add_edge(row+1, col+1)
                        print((str(row+1) + '-' + str(col+1)))
                else:
                    if (str(col+1) + '-' + str(row+1)) not in self.graphicsScene.line_dict.keys():
                        self.graphicsScene.add_edge(col+1, row+1)
                        print((str(col+1) + '-' + str(row+1)))
        elif state == 0:
            if self.edge_matrix[col][row].checkState() != self.edge_matrix[row][col].checkState():
                self.edge_matrix[col][row].setCheckState(0)
                edge = self.build_edge_msg(start_pose, target_pose, 1.0, False)
                if edge not in self.sense_msg.edges:
                    self.sense_msg.edges.append(edge)
                edge = self.build_edge_msg(target_pose, start_pose, 1.0, False)
                if edge not in self.sense_msg.edges:
                    self.sense_msg.edges.append(edge)
                if row < col:
                    if (str(row+1) + '-' + str(col+1)) in self.graphicsScene.line_dict.keys():
                        self.graphicsScene.remove_edge(str(row+1) + '-' + str(col+1))
                else:
                    if (str(col+1) + '-' + str(row+1)) in self.graphicsScene.line_dict.keys():
                        self.graphicsScene.remove_edge(str(col+1) + '-' + str(row+1))

        self.signalEdgeChanged.emit(True)

    def from_dict_to_pose_msg(self, roi_label):
        pose_msg = Pose()
        pose_msg.position.x = self.FTS.region_of_interest[roi_label]['pose']['position'][0]
        pose_msg.position.y = self.FTS.region_of_interest[roi_label]['pose']['position'][1]
        pose_msg.position.z = self.FTS.region_of_interest[roi_label]['pose']['position'][2]
        pose_msg.orientation.w = self.FTS.region_of_interest[roi_label]['pose']['orientation'][0]
        pose_msg.orientation.x = self.FTS.region_of_interest[roi_label]['pose']['orientation'][1]
        pose_msg.orientation.y = self.FTS.region_of_interest[roi_label]['pose']['orientation'][2]
        pose_msg.orientation.z = self.FTS.region_of_interest[roi_label]['pose']['orientation'][3]

        return pose_msg

    def build_edge_msg(self, start_pose, target_pose, weight, add):
        edge = Edge()
        edge.start_pose = start_pose
        edge.target_pose = target_pose
        edge.add.data = add
        edge.weight = weight

        return edge

    def build_roi_msg(self, label):
        roi = Roi()
        string_msg = String()
        string_msg.data = label

        roi.label = string_msg
        roi.pose = self.from_dict_to_pose_msg(label)

        roi.propos_satisfied.append(string_msg)
        print(roi)
        return roi
