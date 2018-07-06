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

from rqt_simulation_msgs.msg import Sense, Edge, Roi
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from MapUtiles import MapUtiles
from generalAP_dialog import GeneralAP_dialog

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QDialog, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QVBoxLayout, QGridLayout, QRadioButton, QGroupBox, QCheckBox, QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem, QFileDialog
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QRectF, QSizeF, QLineF, Slot, pyqtSlot, Qt
from python_qt_binding.QtGui import QImageReader, QImage, QPixmap, QMouseEvent, QPen, QBrush, QColor, QFont

class Change_FTS_dialog(QDialog):
    def __init__(self, current_graphicsScene, FTS):
        super(Change_FTS_dialog, self).__init__()
        self.setObjectName('Change_FTS_dialog')

        # Copy graphics scene
        self.graphicsScene = current_graphicsScene
        # Copy FTS
        self.FTS = FTS

        # Load ui file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'change_FTS.ui')
        loadUi(ui_file, self)

        # Publisher for Sense message
        self.sense_pub = rospy.Publisher('/environment', Sense, queue_size=1)

        self.map_utiles = MapUtiles(self.graphicsScene, self.FTS)
        self.map_utiles.init_FTS_matrix(self.FTS)
        self.map_utiles.signalEdgeChanged.connect(self.edge_changed)
        self.map_utiles.signalNewRoi.connect(self.new_roi)

        # Make whole dialog scrollable
        self.scrollAreaWidgetContents.setLayout(self.map_utiles.grid)

        self.button_add_AP.clicked.connect(self.general_ap)

        # Load graphics scene in graphics view
        self.graphicsView.setScene(self.graphicsScene)

    @pyqtSlot(bool)
    def new_roi(self):
        self.FTS.add_region_marker(self.FTS.region_of_interest['r' + str(self.graphicsScene.regionCounter).zfill(2)], 'r' + str(self.graphicsScene.regionCounter).zfill(2), False)
        self.sense_pub.publish(self.map_utiles.sense_msg)

    # Set the edges both ways in FTS matrix
    @pyqtSlot(bool)
    def edge_changed(self):
        self.sense_pub.publish(self.map_utiles.sense_msg)

    @Slot(bool)
    def general_ap(self):
        print 'now'
        generalAP_dialog = GeneralAP_dialog(self.graphicsScene, self.FTS)
        generalAP_dialog.exec_()
        self.sense_pub.publish(generalAP_dialog.sense_msg)

    # Cancel map dialog
    @Slot(bool)
    def on_button_cancel_pressed(self):
        self.on_button_reset_pressed()
        self.accept()
