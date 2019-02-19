# -*- coding: utf-8 -*-

import os
import sys
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from python_qt_binding.QtWidgets import QWidget, QDialog, QGridLayout, QLabel, QLineEdit
from python_qt_binding.QtCore import Qt, Slot

class TemporaryTask_dialog(QDialog):
    def __init__(self):
        super(TemporaryTask_dialog, self).__init__()
        self.setObjectName('TemporaryTask_dialog')

        # Load ui file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'temporary_task.ui')
        loadUi(ui_file, self)

        self.button_add.clicked.connect(self.add_eventually)
        self.button_remove.clicked.connect(self.remove_eventually)
        self.button_cancel.clicked.connect(self.cancel)
        self.button_send.clicked.connect(self.send_task)

        # Make whole dialog scrollable
        self.grid = QGridLayout()
        self.scrollAreaWidgetContents.setLayout(self.grid)

        self.atomic_propositions = []

        self.eventually_label_list = []
        eventually_label = QLabel('<> (')
        self.eventually_label_list.append(eventually_label)
        self.grid.addWidget(eventually_label, 0, 0)

        self.eventually_input_list = []
        eventually_input = QLineEdit()
        self.eventually_input_list.append(eventually_input)
        self.grid.addWidget(eventually_input, 0 , 1)

        self.end_bracket_list = []
        end_bracket = QLabel(')')
        self.end_bracket_list.append(end_bracket)
        self.grid.addWidget(end_bracket, 0, 2)

    @Slot(bool)
    def add_eventually(self):
        for i in range(0, len(self.end_bracket_list)):
            self.grid.removeWidget(self.end_bracket_list[0])
            self.end_bracket_list[0].deleteLater()
            del self.end_bracket_list[0]

        self.end_bracket_list = []

        eventually_label = QLabel('& <> (')
        position = len(self.eventually_label_list) + len(self.eventually_input_list)
        self.eventually_label_list.append(eventually_label)
        self.grid.addWidget(eventually_label, 0, position + 1)

        eventually_input = QLineEdit()
        self.eventually_input_list.append(eventually_input)
        self.grid.addWidget(eventually_input, 0 , position + 2)

        for i in range(0, len(self.eventually_label_list)):
            end_bracket = QLabel(')')
            self.end_bracket_list.append(end_bracket)
            self.grid.addWidget(end_bracket, 0, position+3+i)

    @Slot(bool)
    def remove_eventually(self):
        for i in range(0, len(self.end_bracket_list)):
            self.grid.removeWidget(self.end_bracket_list[0])
            self.end_bracket_list[0].deleteLater()
            del self.end_bracket_list[0]

        self.end_bracket_list = []

        self.grid.removeWidget(self.eventually_label_list[-1])
        self.eventually_label_list[-1].deleteLater()
        del self.eventually_label_list[-1]

        self.grid.removeWidget(self.eventually_input_list[-1])
        self.eventually_input_list[-1].deleteLater()
        del self.eventually_input_list[-1]

        position = len(self.eventually_label_list) + len(self.eventually_input_list)

        for i in range(0, len(self.eventually_label_list)):
            end_bracket = QLabel(')')
            self.end_bracket_list.append(end_bracket)
            self.grid.addWidget(end_bracket, 0, position+1+i)

    @Slot(bool)
    def send_task(self):
        self.atomic_propositions = []
        for i in range(0, len(self.eventually_input_list)):
            self.atomic_propositions.append(self.eventually_input_list[i].text())
        self.T_des = float(self.T_des_lineEdit.text())
        self.accept()

    @Slot(bool)
    def cancel(self):
        self.accept()



