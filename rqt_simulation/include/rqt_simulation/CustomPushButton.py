# -*- coding: utf-8 -*-

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsTextItem, QGraphicsLineItem, QComboBox, QPushButton
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QLineF, pyqtSlot, Qt
from python_qt_binding.QtGui import QPen, QFont

class CustomPushButton(QPushButton):
    signalPushedButton = pyqtSignal(int)
    def __init__(self, id):
        super(CustomPushButton, self).__init__()

        self.id = id
        self.clicked.connect(self.send_id)

    def send_id(self)
        self.signalPushedButton.emit(self.id)
