# -*- coding: utf-8 -*-

import os
import sys
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from python_qt_binding.QtWidgets import QWidget, QDialog
from python_qt_binding.QtCore import Qt, Slot

class AddAP_dialog(QDialog):
    def __init__(self):
        super(AddAP_dialog, self).__init__()
        self.setObjectName('AddAP_dialog')

        # Load ui file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'add_ap.ui')
        loadUi(ui_file, self)

        self.button_save.clicked.connect(self.save_ap)
        self.button_cancel.clicked.connect(self.cancel)

    @Slot(bool)
    def save_ap(self):
        self.new_ap = self.lineEdit.text()
        self.accept()

    @Slot(bool)
    def cancel(self):
        self.new_ap = False
        self.accept()
