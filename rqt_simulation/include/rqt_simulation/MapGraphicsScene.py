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
        self.line_dict = {}
        self.items_dict = {}

    # Transfrom pixel coordinates to world coordinates
    def pixelToWorld(self, pixel_coords = QPointF()):
        world_coords = ((pixel_coords.x() - self.worldOrigin.x()) * self.map_resolution, -(pixel_coords.y() - self.worldOrigin.y()) * self.map_resolution, 0.0)
        return world_coords

    # Transform world coordinates to pixel coordinates
    def worldToPixel(self, world_coords):
        pixel_coords = QPointF(world_coords[0] / self.map_resolution + self.worldOrigin.x(), -world_coords[1] / self.map_resolution + self.worldOrigin.y())
        return pixel_coords

    # Add ROI to graphics scene
    def add_ROI(self, pixel_coords):
        self.regionCounter += 1

        markerSize = 13
        ellipse_item = QGraphicsEllipseItem(QRectF(QPointF(pixel_coords.x() - markerSize/2, pixel_coords.y() - markerSize/2), QSizeF(markerSize, markerSize)))
        ellipse_item.setBrush(QBrush(QColor('red')))
        self.addItem(ellipse_item)

        label_font = QFont()
        label_font.setPointSize(15)
        region_string = 'r' + str(self.regionCounter).zfill(2)
        ellipse_item_label = QGraphicsTextItem(region_string)
        ellipse_item_label.setPos(pixel_coords)
        ellipse_item_label.setFont(label_font)
        self.addItem(ellipse_item_label)

        self.items_dict.update({region_string : {'ellipse_item' : ellipse_item, 'ellipse_item_label' : ellipse_item_label, 'pixel_coords' : pixel_coords}})

    # Remove las added ROI
    def remove_ROI(self):
        region_string = 'r' + str(self.regionCounter).zfill(2)
        self.removeItem(self.items_dict[region_string]['ellipse_item'])
        self.removeItem(self.items_dict[region_string]['ellipse_item_label'])
        self.removeArrow(self.items_dict[region_string]['arrow'])

        del self.items_dict[region_string]
        self.regionCounter = self.regionCounter - 1

    # Add line between to ROI's
    def add_edge(self, roi_num_1, roi_num_2):
        pixel_coords_1 = self.items_dict['r' + str(roi_num_1).zfill(2)]['pixel_coords']
        pixel_coords_2 = self.items_dict['r' + str(roi_num_2).zfill(2)]['pixel_coords']
        self.line_dict[(str(roi_num_1) + '-' + str(roi_num_2))] = QGraphicsLineItem(QLineF(pixel_coords_2, pixel_coords_1))
        self.addItem(self.line_dict[(str(roi_num_1) + '-' + str(roi_num_2))])

    # Remove line between to ROI's
    def remove_edge(self, edge):
        self.removeItem(self.line_dict[edge])
        del self.line_dict[edge]

    # Reset graphics scene
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
        #self.arrow_list = []

    # Load map
    def load_map(self, scenario):
        self.scenario = scenario
        map_yaml = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'scenarios', scenario, 'map.yaml')
        self.loadConfig(map_yaml)
        if scenario == 'pal_office' or scenario == 'sml':
            map = 'map.pgm'
        else:
            map = 'map.png'

        map_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'scenarios', scenario, map)
        pixmap = QPixmap(map_file)
        self.mapSize = pixmap.size()
        self.addPixmap(pixmap)

        # Add world origin
        self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + self.mapSize.height())
        self.addCoordinateSystem(self.worldOrigin, 0.0)

    # Send signal if mouse button is pressed
    def mousePressEvent(self, event):
        pos = event.lastScenePos()
        self.signalMousePressedPos.emit(pos)

    # Send signal if mouse button is released
    def mouseReleaseEvent(self, event):
        pos = event.lastScenePos()
        self.signalMouseReleasedPos.emit(pos)

    # Send signal if mouse is moving in graphic scene
    def mouseMoveEvent(self, event):
        pos = event.lastScenePos()
        self.signalMouseMovePos.emit(pos)

    # Add arrow to graphic scene
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

    # Remove arrow from graphics scene
    def removeArrow(self, arrow):
        for n in arrow:
            self.removeItem(n)

    # Add coordinate system
    def addCoordinateSystem(self, origin = QPointF(), angle = 0.0):
        XAxis = QPointF(origin.x() + 100, origin.y())
        YAxis = QPointF(origin.x(), origin.y() - 100)

        self.addArrow(origin, XAxis)
        self.addArrow(origin, YAxis)
        XLabel = self.addText('X', QFont())
        XLabel.setPos(XAxis)
        YLabel = self.addText('Y', QFont())
        YLabel.setPos(YAxis)

    # Load the data from map.yaml file
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
        qualisys = data['qualisys']
        if qualisys:
            self.tf_qualisys_to_map = data['tf_qualisys_to_map']
            rospy.loginfo('rqt_simulation map tf to qualisys : %s' % (self.tf_qualisys_to_map))
        rospy.loginfo('rqt_simulation map : %s' % (self.scenario))
        rospy.loginfo('rqt_simulation map resolution : %.6f' % (self.map_resolution))
        rospy.loginfo('rqt_simulation map origin : %s' % (self.map_origin,))
        rospy.loginfo('rqt_simulation map negate : %s' % (self.map_negate))
        rospy.loginfo('rqt_simulation map occupied threshold : %s' % (self.map_occupied_thresh))
        rospy.loginfo('rqt_simulation map free threshold : %s' % (self.map_free_thresh))

