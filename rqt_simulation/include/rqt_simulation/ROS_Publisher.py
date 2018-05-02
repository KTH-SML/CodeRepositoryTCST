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
from std_msgs.msg import Bool, String

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QApplication, QGraphicsScene, QGraphicsTextItem, QVBoxLayout, QComboBox, QLineEdit, QTextBrowser
from python_qt_binding.QtCore import QTimer, Slot, pyqtSlot, QSignalMapper, QRectF, QPointF
from python_qt_binding.QtGui import QImageReader, QImage, QMouseEvent, QCursor, QBrush, QColor, QPixmap, QTransform, QFont

class ROS_Publisher(QWidget):
    def __init__(self):
        super(ROS_Publisher, self).__init__()
        self.id_counter = 0
        self.publisher_dict = {}
        self._timeout_mapper = QSignalMapper(self)
        self._timeout_mapper.mapped[int].connect(self.publish_once)

    def add_publisher(self, topic, type, rate, msg):

        publisher = {}
        publisher['publisher_id'] = self.id_counter
        publisher['message'] = msg
        publisher['publisher'] = rospy.Publisher(topic, type, queue_size=1)
        self.publisher_dict[publisher['publisher_id']] = publisher
        publisher['timer'] = QTimer(self)
        self._timeout_mapper.setMapping(publisher['timer'], publisher['publisher_id'])
        publisher['timer'].timeout.connect(self._timeout_mapper.map)
        publisher['timer'].start(int(1000.0 / rate))
        self.id_counter += 1

    def remove_publisher(self, id):
        del self.publisher_dict[publisher['publisher_id']]

    @Slot(int)
    def publish_once(self, publisher_id):
        publisher = self.publisher_dict.get(publisher_id, None)
        if publisher is not None:
            publisher['publisher'].publish(publisher['message'])
