# -*- coding: utf-8 -*-
'''
This is a custom QPushbutton. If the the button clicked
it emits the robot tab id

Input:  id  int

Output: robot tab id    int
'''

from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot, Qt

class CustomPushButton(QPushButton):
    signalPushedButton = pyqtSignal(int)
    def __init__(self, id):
        super(CustomPushButton, self).__init__()

        self.id = id
        self.clicked.connect(self.send_id)

    def send_id(self)
        self.signalPushedButton.emit(self.id)
