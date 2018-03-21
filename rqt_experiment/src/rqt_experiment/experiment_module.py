#!/usr/bin/env python
import os
import rospy
import rospkg
import argparse
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray

from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal, Slot, QSignalMapper, QTimer
from qt_gui.plugin import Plugin
#from python_qt_binding import loadUi
#from python_qt_binding.QtWidgets import QWidget
from .experiment_widget import ExperimentWidget

class ExperimentPlugin(Plugin):

    def __init__(self, context):
        super(ExperimentPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ExperimentPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = ExperimentWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        #ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'resource', 'SimulationPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        #loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        #self._widget.setObjectName('SimulationPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        point_list = []
        point = Point()
        point.x = 0.0
        point.y = 0.0
        point.z = 0.0
        point_list.append(point)

        pose = Pose()



    def shutdown_plugin(self):
        #self._widget.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
