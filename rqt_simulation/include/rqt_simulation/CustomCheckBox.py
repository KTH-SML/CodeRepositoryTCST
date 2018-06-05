# -*- coding: utf-8 -*-
'''
This is a custom QCheckBox. It emits state, row and column index
if the check box state is changed.

Input:  label   string
        row     int
        column  int

signalStateChanged: state   int
                    row     int
                    column  int
'''

from python_qt_binding.QtWidgets import QCheckBox
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot, Qt

class CustomCheckBox(QCheckBox):
    signalStateChanged = pyqtSignal(int, int, int)
    def __init__(self, label, row, column):
        super(CustomCheckBox, self).__init__(label)

        self.row = row
        self.column = column
        self.stateChanged.connect(self.send_row_and_col)

    def send_row_and_col(self, state):
        self.signalStateChanged.emit(state, self.row, self.column)
