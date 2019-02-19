# -*- coding: utf-8 -*-

import rospy

from math import atan2, cos, sin, pi, atan

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsTextItem
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QLineF, pyqtSlot, Qt, QObject
from python_qt_binding.QtGui import QPen, QFont

class ROS_Subscriber(QObject):
    received = pyqtSignal(int)
    def __init__(self, topic, type, cb):
        super(ROS_Subscriber, self).__init__()

        self.subscriber = rospy.Subscriber(topic, type, cb, topic)
        self.topic = topic
