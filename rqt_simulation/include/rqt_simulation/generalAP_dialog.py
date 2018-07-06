# -*- coding: utf-8 -*-

import os
import sys
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QGridLayout, QVBoxLayout, QGroupBox
from python_qt_binding.QtCore import Qt, Slot, pyqtSlot

from AddAP_dialog import AddAP_dialog
from CustomCheckBox import CustomCheckBox
from MapUtiles import MapUtiles

from rqt_simulation_msgs.msg import Sense, Edge, Roi
from std_msgs.msg import String

class GeneralAP_dialog(QDialog):
    def __init__(self, graphicsScene, FTS):
        super(GeneralAP_dialog, self).__init__()
        self.setObjectName('GeneralAP_dialog')

        # Copy FTS
        self.FTS = FTS

        # Copy graphicsScene
        self.graphicsScene = graphicsScene
        self.map_utiles = MapUtiles(self.graphicsScene, self.FTS)

        # List with AP's
        self.ap_list = []

        # Load ui file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'general_ap.ui')
        loadUi(ui_file, self)

        self.sense_msg = Sense()

        # Sort the ROI's alphabetic, needed for FTS matrix since it's implemented with list
        self.sorted_keys = self.FTS.region_of_interest.keys()
        self.sorted_keys.sort()

        # Set dialog layout to grid
        self.grid = QGridLayout()

        # Initialize variables for FTS matrix
        self.groupBox_list = []
        self.ap_matrix = []
        self.vbox_list = []
        self.vbox = QVBoxLayout()

        for i in range(0, len(self.FTS.region_of_interest)):
            groupBox = QGroupBox(self.sorted_keys[i])
            self.groupBox_list.append(groupBox)
            checkBox_list = []
            self.ap_matrix.append(checkBox_list)

            vbox = QVBoxLayout()
            self.vbox_list.append(vbox)

            self.groupBox_list[i].setLayout(self.vbox_list[i])
            self.grid.addWidget(self.groupBox_list[i], 0, i+1, Qt.AlignRight)

            for j in range(0, len(self.FTS.region_of_interest[self.sorted_keys[i]]['propos'])):
                if (self.FTS.region_of_interest[self.sorted_keys[i]]['propos'][j] not in self.ap_list) and (self.FTS.region_of_interest[self.sorted_keys[i]]['propos'][j] != self.sorted_keys[i]):
                    print(self.FTS.region_of_interest[self.sorted_keys[i]]['propos'][j])
                    self.ap_list.append(self.FTS.region_of_interest[self.sorted_keys[i]]['propos'][j])

        for i in range(0, len(self.FTS.region_of_interest)):
            for j in range(0, len(self.ap_list)):
                self.ap_matrix[i].append(CustomCheckBox(self.ap_list[j], i, j))
                self.vbox_list[i].addWidget(self.ap_matrix[i][-1])
                if self.ap_list[j] in self.FTS.region_of_interest[self.sorted_keys[i]]['propos']:
                    self.ap_matrix[i][j].setCheckState(2)
                self.ap_matrix[i][-1].signalStateChanged.connect(self.add_textItem)

        # Make whole dialog scrollable
        self.scrollAreaWidgetContents.setLayout(self.grid)

        self.button_add_ap.clicked.connect(self.add_ap)
        self.button_save.clicked.connect(self.save)

        self.sense_msg = Sense()

    @Slot(bool)
    def add_ap(self):
        add_ap_dialog = AddAP_dialog()
        add_ap_dialog.exec_()

        if add_ap_dialog.new_ap:
            print(add_ap_dialog.new_ap)
            for i in range(0, len(self.FTS.region_of_interest)):
                self.ap_matrix[i].append(CustomCheckBox(add_ap_dialog.new_ap, i, len(self.ap_list)))
                self.ap_matrix[i][-1].signalStateChanged.connect(self.add_textItem)
                self.vbox_list[i].addWidget(self.ap_matrix[i][-1])

            self.ap_list.append(add_ap_dialog.new_ap)

    @pyqtSlot(int, int, int)
    def add_textItem(self, state, col, row):
        if state == 2:
            self.graphicsScene.add_ap('r' + str(col+1).zfill(2), self.ap_list[row])
        if state == 0:
            self.graphicsScene.remove_ap('r' + str(col+1).zfill(2), self.ap_list[row])

    @Slot(bool)
    def save(self):
        print(self.ap_list)
        for i in range(0, len(self.FTS.region_of_interest)):
            for j in range(0, len(self.ap_list)):
                roi = self.map_utiles.build_roi_msg(self.sorted_keys[i])
                ap_changed = False
                print(self.ap_list[j])
                print(self.FTS.region_of_interest[self.sorted_keys[i]]['propos'])
                if (self.ap_matrix[i][j].checkState() == 2) and (self.ap_list[j] not in self.FTS.region_of_interest[self.sorted_keys[i]]['propos']):
                    print('---new ap---')
                    print(self.ap_list[j])
                    string_msg = String()
                    string_msg.data = str(self.ap_list[j])
                    roi.propos_satisfied.append(string_msg)
                    ap_changed = True
                    print('---end---')
                elif (self.ap_matrix[i][j].checkState() == 0) and (self.ap_list[j] in self.FTS.region_of_interest[self.sorted_keys[i]]['propos']):
                    string_msg = String()
                    string_msg.data = str(self.ap_list[j])
                    roi.propos_unsatisfied.append(string_msg)
                    ap_changed = True
                if ap_changed:
                    self.sense_msg.rois.append(roi)


            del self.FTS.region_of_interest[self.sorted_keys[i]]['propos'][1:]
            for j in range(0, len(self.ap_list)):
                if (self.ap_matrix[i][j].checkState() == 2):
                    self.FTS.add_propos(self.sorted_keys[i], self.ap_list[j])
        print(self.sense_msg)
        self.accept()


