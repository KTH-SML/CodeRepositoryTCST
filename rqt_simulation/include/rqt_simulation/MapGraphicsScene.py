#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import atan2, cos, sin, pi, atan
import os
import rospkg
import yaml
import rospy

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsTextItem, QGraphicsLineItem, QGraphicsEllipseItem
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QLineF, pyqtSlot, Qt, QRectF, QSizeF
from python_qt_binding.QtGui import QPen, QFont, QBrush, QColor, QPixmap

class MapGraphicsScene(QGraphicsScene):
    signalMousePressedPos = pyqtSignal(QPointF)
    signalMouseReleasedPos = pyqtSignal(QPointF)
    signalMouseMovePos = pyqtSignal(QPointF)
    def __init__(self):
        super(MapGraphicsScene, self).__init__()
        self.regionCounter = 0
        self.ellipse_items = []
        self.ellipse_items_labels = []
        self.pixel_coords_list = []
        self.line_dict = {}
        self.arrow_list = []

    def pixelToWorld(self, pixel_coords = QPointF()):
        world_coords = ((pixel_coords.x() - self.worldOrigin.x()) * self.map_resolution, -(pixel_coords.y() - self.worldOrigin.y()) * self.map_resolution, 0.0)
        return world_coords

    def worldToPixel(self, world_coords):
        pixel_coords = QPointF(world_coords[0] / self.map_resolution + self.worldOrigin.x(), -world_coords[1] / self.map_resolution + self.worldOrigin.y())
        return pixel_coords

    def add_ROI(self, pixel_coords):
        self.regionCounter += 1
        self.pixel_coords_list.append(pixel_coords)
        markerSize = 13
        self.ellipse_items.append(QGraphicsEllipseItem(QRectF(QPointF(pixel_coords.x() - markerSize/2, pixel_coords.y() - markerSize/2), QSizeF(markerSize, markerSize))))
        self.ellipse_items[self.regionCounter - 1].setBrush(QBrush(QColor('red')))
        self.addItem(self.ellipse_items[self.regionCounter - 1])

        label_font = QFont()
        label_font.setPointSize(15)
        regionString = 'r' + str(self.regionCounter).zfill(2)
        self.ellipse_items_labels.append(QGraphicsTextItem(regionString))
        self.ellipse_items_labels[self.regionCounter - 1].setPos(pixel_coords)
        self.ellipse_items_labels[self.regionCounter - 1].setFont(label_font)
        self.addItem(self.ellipse_items_labels[self.regionCounter - 1])

    def remove_ROI(self):
        self.removeItem(self.ellipse_items[self.regionCounter-1])
        self.removeItem(self.ellipse_items_labels[self.regionCounter-1])
        self.removeArrow(self.arrow_list[self.regionCounter-1])
        del self.arrow_list[self.regionCounter-1]
        del self.ellipse_items[self.regionCounter-1]
        del self.ellipse_items_labels[self.regionCounter-1]
        del self.pixel_coords_list[self.regionCounter-1]

        self.regionCounter = self.regionCounter - 1

    def add_edge(self, roi_num_1, roi_num_2):
        self.line_dict[(str(roi_num_1) + '-' + str(roi_num_2))] = QGraphicsLineItem(QLineF(self.pixel_coords_list[roi_num_2 - 1], self.pixel_coords_list[roi_num_1 - 1]))
        self.addItem(self.line_dict[(str(roi_num_1) + '-' + str(roi_num_2))])

    def remove_edge(self, edge):
        self.removeItem(self.line_dict[edge])
        del self.line_dict[edge]

    def reset(self):
        for i in range(0, self.regionCounter):
            self.remove_ROI()
        for i in range(0, len(self.line_dict)):
            self.remove_edge(self.line_dict.keys()[0])

        self.regionCounter = 0
        self.ellipse_items = []
        self.ellipse_items_labels = []
        self.pixel_coords_list = []
        self.line_dict = {}
        self.arrow_list = []

    def load_map(self, scenario):
        self.scenario = scenario
        map_yaml = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', scenario, 'map.yaml')
        self.loadConfig(map_yaml)
        if scenario == 'pal_office' or scenario == 'sml':
            map = 'map.pgm'
        else:
            map = 'map.png'

        map_file = os.path.join(rospkg.RosPack().get_path('c4r_simulation'), 'scenarios', scenario, map)
        pixmap = QPixmap(map_file)
        self.mapSize = pixmap.size()
        self.addPixmap(pixmap)

        # Add world origin
        self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + self.mapSize.height())
        self.addCoordinateSystem(self.worldOrigin, 0.0)

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

    def loadConfig(self, filename):
        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()
        self.map_image = data['image']
        self.map_resolution = data['resolution']
        self.map_origin = tuple(data['origin'])
        self.map_negate = data['negate']
        self.map_occupied_thresh = data['occupied_thresh']
        self.map_free_thresh = data['free_thresh']
        rospy.loginfo('rqt_simulation map : %s' % (self.scenario))
        rospy.loginfo('rqt_simulation map resolution : %.6f' % (self.map_resolution))
        rospy.loginfo('rqt_simulation map origin : %s' % (self.map_origin,))
        rospy.loginfo('rqt_simulation map negate : %s' % (self.map_negate))
        rospy.loginfo('rqt_simulation map occupied threshold : %s' % (self.map_occupied_thresh))
        rospy.loginfo('rqt_simulation map free threshold : %s' % (self.map_free_thresh))
