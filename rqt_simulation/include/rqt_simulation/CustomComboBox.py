# -*- coding: utf-8 -*-
'''
This is a custom QCombobox. If the current index changed it emits
index of the box, id of robot tab

Input:  id  int

Output: robot tab id    int
        box index       int
'''

from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot, Qt

class CustomComboBox(QComboBox):
    signalIndexChanged = pyqtSignal(int, int)
    def __init__(self, id):
        super(CustomComboBox, self).__init__()

        self.id = id
        self.currentIndexChanged.connect(self.send_index_and_id)

    def send_index_and_id(self, index):
        self.signalIndexChanged.emit(index, self.id)


