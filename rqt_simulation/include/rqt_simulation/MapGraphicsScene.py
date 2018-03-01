#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import atan2, cos, sin, pi, atan

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsTextItem, QGraphicsLineItem
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QLineF, pyqtSlot, Qt
from python_qt_binding.QtGui import QPen, QFont

class MapGraphicsScene(QGraphicsScene):
    signalMousePressedPos = pyqtSignal(QPointF)
    signalMouseReleasedPos = pyqtSignal(QPointF)
    signalMouseMovePos = pyqtSignal(QPointF)
    def __init__(self):
        super(MapGraphicsScene, self).__init__()
        #self.arrowItems = []

    def mousePressEvent(self, event):
        pos = event.lastScenePos()
        self.signalMousePressedPos.emit(pos)

    def mouseReleaseEvent(self, event):
        pos = event.lastScenePos()
        self.signalMouseReleasedPos.emit(pos)

    def mouseMoveEvent(self, event):
        pos = event.lastScenePos()
        self.signalMouseMovePos.emit(pos)

    def addArrow(self, startPoint = QPointF(), endPoint = QPointF(), pen = QPen()):
        alpha = 5*pi/6
        arrowLength = 10

        theta = atan2((endPoint.y() - startPoint.y()) , (endPoint.x() - startPoint.x()))

        gamma1 = theta + alpha
        gamma2 = theta - alpha
        arrowPoint_1 = QPointF(endPoint.x() + arrowLength * cos(gamma1), endPoint.y() + arrowLength * sin(gamma1))
        arrowPoint_2 = QPointF(endPoint.x() + arrowLength * cos(gamma2), endPoint.y() + arrowLength * sin(gamma2))
        line_0 = QLineF(startPoint, endPoint)
        line_1 = QLineF(endPoint,arrowPoint_1)
        line_2 = QLineF(endPoint,arrowPoint_2)

        line_item_0 = QGraphicsLineItem(line_0)
        line_item_0.setPen(pen)
        line_item_1 = QGraphicsLineItem(line_1)
        line_item_1.setPen(pen)
        line_item_2 = QGraphicsLineItem(line_2)
        line_item_2.setPen(pen)

        arrowItems = [line_item_0, line_item_1, line_item_2]
        #self.arrowItems.append(arrowItems)

        self.addItem(line_item_0)
        self.addItem(line_item_1)
        self.addItem(line_item_2)

        return arrowItems

    def removeArrow(self, arrow):
        for n in arrow:
            self.removeItem(n)

    def addCoordinateSystem(self, origin = QPointF(), angle = 0.0):
        XAxis = QPointF(origin.x() + 100, origin.y())
        YAxis = QPointF(origin.x(), origin.y() - 100)

        self.addArrow(origin, XAxis)
        self.addArrow(origin, YAxis)
        XLabel = self.addText('X', QFont())
        XLabel.setPos(XAxis)
        YLabel = self.addText('Y', QFont())
        YLabel.setPos(YAxis)
