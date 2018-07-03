#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is a QGraphicsScene to load the map and draw the FTS
'''

from math import atan2, cos, sin, pi, atan
from pyquaternion import Quaternion
import os
import rospkg
import yaml
import rospy

from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsTextItem, QGraphicsLineItem, QGraphicsEllipseItem
from python_qt_binding.QtCore import QTimer, QEvent, pyqtSignal, QPointF, QLineF, pyqtSlot, Qt, QRectF, QSizeF
from python_qt_binding.QtGui import QPen, QFont, QBrush, QColor, QPixmap, QTransform

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
    # Input:    pixel coordinates   QPointF
    # Output:   world coordinates   (x, y, z)
    def pixelToWorld(self, pixel_coords = QPointF()):
        world_coords = ((pixel_coords.x() - self.worldOrigin.x()) * self.map_resolution, -(pixel_coords.y() - self.worldOrigin.y()) * self.map_resolution, 0.0)
        return world_coords

    # Transform world coordinates to pixel coordinates
    # Input:    world coordinates   (x, y, z)
    # Output:   pixel coordinates   QPointF
    def worldToPixel(self, world_coords):
        pixel_coords = QPointF(world_coords[0] / self.map_resolution + self.worldOrigin.x(), -world_coords[1] / self.map_resolution + self.worldOrigin.y())
        return pixel_coords

    # Add ROI to graphics scene
    # Input:    pixel coordinates   QPointF
    # Update GraphicsScene
    def add_ROI(self, pixel_coords):
        self.regionCounter += 1

        markerSize = 25
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

        self.items_dict.update({region_string : {'ellipse_item' : ellipse_item, 'ellipse_item_label' : ellipse_item_label, 'pixel_coords' : pixel_coords, 'ap_item_label' : {}}})

    # Remove las added ROI
    # Update GraphicsScene
    def remove_ROI(self):
        region_string = 'r' + str(self.regionCounter).zfill(2)
        self.removeItem(self.items_dict[region_string]['ellipse_item'])
        self.removeItem(self.items_dict[region_string]['ellipse_item_label'])
        self.removeArrow(self.items_dict[region_string]['arrow'])
        for i in range(0, len(self.items_dict[region_string]['ap_item_label'].keys())):
            self.remove_ap(region_string, self.items_dict[region_string]['ap_item_label'].keys()[i])

        del self.items_dict[region_string]
        self.regionCounter = self.regionCounter - 1

    # Add line between to ROI's
    # Input:    number ROI 1    int
    #           number ROI 2    int
    # Update GraphicsScene
    def add_edge(self, roi_num_1, roi_num_2):
        pixel_coords_1 = self.items_dict['r' + str(roi_num_1).zfill(2)]['pixel_coords']
        pixel_coords_2 = self.items_dict['r' + str(roi_num_2).zfill(2)]['pixel_coords']
        self.line_dict[(str(roi_num_1) + '-' + str(roi_num_2))] = QGraphicsLineItem(QLineF(pixel_coords_2, pixel_coords_1))
        self.addItem(self.line_dict[(str(roi_num_1) + '-' + str(roi_num_2))])

    # Remove line between to ROI's
    # Input:    edge label  string      e.g. 1-2
    # Update GraphicsScene
    def remove_edge(self, edge):
        self.removeItem(self.line_dict[edge])
        del self.line_dict[edge]

    # Add general atomic proposition label
    # Input:    region label    string
    #           ap label        string
    # Update GraphicsScene
    def add_ap(self, region, ap):
        label_font = QFont()
        label_font.setPointSize(15)
        ap_item_label = QGraphicsTextItem(ap)
        ap_item_label.setPos(QPointF(self.items_dict[region]['pixel_coords'].x()-25, self.items_dict[region]['pixel_coords'].y()))
        ap_item_label.setFont(label_font)
        self.addItem(ap_item_label)

        self.items_dict[region]['ap_item_label'].update({ap : ap_item_label})

    # Remove general atomic proposition label
    # Input:    region label    string
    #           ap label        string
    # Update GraphicsScene
    def remove_ap(self, region, ap):
        self.removeItem(self.items_dict[region]['ap_item_label'][ap])

    # Reset graphics scene
    # Update GraphicsScene
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
    # Input:    scenario name   string
    # Update GraphicsScene
    def load_map(self, scenario):
        self.scenario = scenario
        map_yaml = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'scenarios', scenario, 'map.yaml')
        self.loadConfig(map_yaml)
        map = 'map.png'

        map_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'scenarios', scenario, map)
        pixmap = QPixmap(map_file)
        self.mapSize = pixmap.size()
        self.addPixmap(pixmap)

        # Add world origin
        self.worldOrigin = QPointF(-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + self.mapSize.height())
        self.addCoordinateSystem(self.worldOrigin, 0.0)

    # Send signal if mouse button is pressed
    # Input:    click event
    # Output:   pixel coordinates   QPointF
    def mousePressEvent(self, event):
        pos = event.lastScenePos()
        self.signalMousePressedPos.emit(pos)

    # Send signal if mouse button is released
    # Input:    release event
    # Output:   pixel coordinates   QPointF
    def mouseReleaseEvent(self, event):
        pos = event.lastScenePos()
        self.signalMouseReleasedPos.emit(pos)

    # Send signal if mouse is moving in graphic scene
    # Input:    move event
    # Output:   pixel coordinates   QPointF
    def mouseMoveEvent(self, event):
        pos = event.lastScenePos()
        self.signalMouseMovePos.emit(pos)

    # Add arrow to graphic scene
    # Input:    start point     QPointF
    #           end point       QPointF
    #           drawing options QPen
    # Output:   arrow           list with 3 QGraphicsLineItem
    # Update GraphicsScene
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
    # Input:    arrow   list with 3 QGraphicsLineItem
    # Update GraphicsScene
    def removeArrow(self, arrow):
        for n in arrow:
            self.removeItem(n)


    def scale_map(self, graphicsView, scenario):
        rectF = graphicsView.geometry()
        if (float(rectF.width())/self.mapSize.width() < float(rectF.height())/self.mapSize.height()):
            scale = float(rectF.width())/self.mapSize.width()
        elif scenario == 'pal_office' or scenario == 'sml':
            scale = 0.7
        else:
            scale = float(rectF.height())/self.mapSize.height()
        transform = QTransform(scale, 0, 0.0, scale, 0, 0)

        return transform

    # Load ROI's and edges from a FTS
    # Input:    FTS     dict
    # Update GraphicsScene
    def load_graphic_from_FTS(self, FTS):
        sorted_keys = FTS.region_of_interest.keys()
        sorted_keys.sort()

        arrow_length = 50

        # Add all the ROI's and edges
        for i in range(0, len(FTS.region_of_interest)):
            region_string = 'r' + str(i+1).zfill(2)
            pixel_coords = self.worldToPixel(FTS.region_of_interest[sorted_keys[i]]['pose']['position'])
            self.add_ROI(pixel_coords)

            for j in range(0, len(FTS.region_of_interest[sorted_keys[i]]['propos'])):
                if sorted_keys[i] != FTS.region_of_interest[sorted_keys[i]]['propos'][j]:
                    self.add_ap(sorted_keys[i], FTS.region_of_interest[sorted_keys[i]]['propos'][j])

            quaternion = Quaternion(FTS.region_of_interest[sorted_keys[i]]['pose']['orientation'])
            rot_axis = quaternion.axis
            theta = quaternion.angle * rot_axis[2]
            end_point = QPointF(pixel_coords.x() + arrow_length * cos(theta), pixel_coords.y() - arrow_length * sin(theta))
            arrow = self.addArrow(pixel_coords, end_point)
            self.items_dict[region_string]['arrow'] = arrow

        # Add all edges to graphics scene
        for i in range(0, len(FTS.region_of_interest)):
            for j in range(0, len(FTS.region_of_interest[sorted_keys[i]]['edges'])):
                index = sorted_keys.index(FTS.region_of_interest[sorted_keys[i]]['edges'][j]['target'])
                if i < index:
                    if (str(i+1) + '-' + str(index+1)) not in self.line_dict.keys():
                        self.add_edge(i+1, index+1)
                else:
                    if (str(index+1) + '-' + str(i+1)) not in self.line_dict.keys():
                        self.add_edge(index+1, i+1)


    # Add coordinate system
    # Input:    origin     QPointF
    #           angle      Float
    # Update GraphicsScene
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
    # Input:    file    string
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

