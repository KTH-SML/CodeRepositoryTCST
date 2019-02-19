# -*- coding: utf-8 -*-

from python_qt_binding.QtGui import QTransform
import roslaunch
import os
import rospkg
import sys

class WidgetUtiles(object):
    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

    def launch_gazebo(self, scenario):
        launch_world = roslaunch.parent.ROSLaunchParent(self.uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'setup_simulation.launch')])
        sys.argv.append('scenario:=' + scenario)
        launch_world.start()
        del sys.argv[2:len(sys.argv)]

    def launch_logger(self):
        launch_logger = roslaunch.parent.ROSLaunchParent(self.uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'rosbag_writer.launch')])
        launch_logger.start()
        del sys.argv[2:len(sys.argv)]

    def set_qualisys_args(self, tf):
        sys.argv.append('trans_x:=' + str(tf['translation'][0]))
        sys.argv.append('trans_y:=' + str(tf['translation'][1]))
        sys.argv.append('trans_z:=' + str(tf['translation'][2]))
        sys.argv.append('orient_w:=' + str(tf['rotation'][0]))
        sys.argv.append('orient_x:=' + str(tf['rotation'][1]))
        sys.argv.append('orient_y:=' + str(tf['rotation'][2]))
        sys.argv.append('orient_z:=' + str(tf['rotation'][3]))

    def launch_qualisys(self):
        launch_qualisys = roslaunch.parent.ROSLaunchParent(self.uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'qualisys_mapper.launch')])
        launch_qualisys.start()
        del sys.argv[2:len(sys.argv)]

