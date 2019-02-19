#!/usr/bin/env python

from qt_gui.plugin import Plugin
from .simulation_widget import SimulationWidget

class SimulationPlugin(Plugin):

    def __init__(self, context):
        super(SimulationPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SimulationPlugin')

        # Create QWidget
        self._widget = SimulationWidget()

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
