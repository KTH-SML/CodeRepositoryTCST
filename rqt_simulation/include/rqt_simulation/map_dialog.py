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

from CustomCheckBox import CustomCheckBox
from generalAP_dialog import GeneralAP_dialog
from MapUtiles import MapUtiles

class Map_dialog(QDialog):
    def __init__(self, current_graphicsScene, FTS):
        super(Map_dialog, self).__init__()
        self.setObjectName('Map_dialog')

        # Copy graphics scene
        self.graphicsScene = current_graphicsScene
        # Copy FTS
        self.FTS = FTS

        # Load ui file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'map.ui')
        loadUi(ui_file, self)

        self.map_utiles = MapUtiles(self.graphicsScene, self.FTS)
        self.map_utiles.init_FTS_matrix(self.FTS)
        self.map_utiles.signalNewRoi.connect(self.new_roi)

        # Make whole dialog scrollable
        self.scrollAreaWidgetContents.setLayout(self.map_utiles.grid)

        # Connect all buttons with corresponding functions
        self.button_save_FTS.pressed.connect(self.on_button_FTS_save_pressed)
        #self.button_save_FTS.setEnabled(False)
        self.button_reset.pressed.connect(self.on_button_reset_pressed)
        self.button_set_edges.pressed.connect(self.on_button_set_edges_pressed)
        self.button_cancel.clicked.connect(self.on_button_cancel_pressed)
        self.button_ROI.clicked.connect(self.remove_last_ROI)
        #self.button_ROI.setEnabled(False)
        self.button_delete_edges.clicked.connect(self.delete_edges)
        self.button_load_FTS.clicked.connect(self.load_FTS)
        self.button_general_AP.clicked.connect(self.general_ap)
        #self.button_set_edges.setEnabled(False)

        # Load graphics scene in graphics view
        self.graphicsView.setScene(self.graphicsScene)

    # Save the current FTS in yaml file
    @Slot(bool)
    def on_button_FTS_save_pressed(self):
        self.button_save_FTS.setEnabled(False)
        sorted_keys = self.FTS.region_of_interest.keys()
        sorted_keys.sort()
        print(sorted_keys)
        for i in range(0, len(self.map_utiles.edge_matrix)):
            self.FTS.region_of_interest[sorted_keys[i]]['edges'] = []
            for j in range(0, len(self.map_utiles.edge_matrix[0])):
                if (self.map_utiles.edge_matrix[i][j].checkState() == 2):
                    self.FTS.add_edge(sorted_keys[i], sorted_keys[j], cost=1.0)

        print('start saving')
        data = {'FTS' : self.FTS.region_of_interest}
        print(data)
        env_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        with codecs.open(env_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(data, outfile, default_flow_style=False)

        self.accept()

    # Reset FTS
    @Slot(bool)
    def on_button_reset_pressed(self):
        print('Reset')
        self.map_utiles.reset_FTS_matrix()
        self.graphicsScene.reset()
        self.FTS.region_of_interest = {}
        self.button_save_FTS.setEnabled(False)

    # Remove last added ROI
    @Slot(bool)
    def remove_last_ROI(self):
        self.map_utiles.remove_FTS_matrix()
        if self.graphicsScene.regionCounter < 1:
            self.button_ROI.setEnabled(False)

    # Sets all edges between ROI's
    @Slot(bool)
    def on_button_set_edges_pressed(self):
        for i in range(0, len(self.map_utiles.edge_matrix)):
            for j in range(0, len(self.map_utiles.edge_matrix[0])):
                self.map_utiles.edge_matrix[i][j].setCheckState(Qt.Checked)

    # Removes all edges between ROI's
    @Slot(bool)
    def delete_edges(self):
        for i in range(0, len(self.map_utiles.edge_matrix)):
            for j in range(0, len(self.map_utiles.edge_matrix[0])):
                self.map_utiles.edge_matrix[i][j].setCheckState(Qt.Unchecked)

    @pyqtSlot(bool)
    def new_roi(self):
        self.button_save_FTS.setEnabled(True)
        self.button_ROI.setEnabled(True)

    @Slot(bool)
    def general_ap(self):
        print 'now'
        generalAP_dialog = GeneralAP_dialog(self.graphicsScene, self.FTS)
        generalAP_dialog.exec_()

    # Cancel map dialog
    @Slot(bool)
    def on_button_cancel_pressed(self):
        self.on_button_reset_pressed()
        self.accept()

    # Load FTS from a yaml file
    @Slot(bool)
    def load_FTS(self):
        # Reset graphics Scene
        self.on_button_reset_pressed()
        self.FTS.load_FTS()
        self.graphicsScene.load_graphic_from_FTS(self.FTS)
        self.map_utiles.init_FTS_matrix(self.FTS)
