# -*- coding: utf-8 -*-

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsTextItem, QGraphicsLineItem, QComboBox
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QLineF, pyqtSlot, Qt
from python_qt_binding.QtGui import QPen, QFont

class CustomComboBox(QComboBox):
    signalIndexChanged = pyqtSignal(int, int)
    def __init__(self, id):
        super(CustomComboBox, self).__init__()

        self.id = id
        self.currentIndexChanged.connect(self.send_index_and_id)

    def send_index_and_id(self, index):
        self.signalIndexChanged.emit(index, self.id)


