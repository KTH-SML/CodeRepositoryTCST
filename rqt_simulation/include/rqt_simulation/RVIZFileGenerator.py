# -*- coding: utf-8 -*-
import yaml
import codecs
import os
import sys
import rospy
import rospkg

class RVIZFileGenerator:
    def __init__(self):
        dict = {}
        Panels = []
        dict_class_rviz_displays = {'Class' : 'rviz/Displays'}
        dict_class_rviz_displays.update({'Help Height' : 78})
        dict_class_rviz_displays.update({'Name' : 'Displays'})

        Expanded = ['/Global Options1',
                    '/Robot1',
                    '/Robot1/RobotModel1',
                    '/Robot1/Dynamics1',
                    '/Robot1/Footprint1',
                    '/Robot1/Dynamic Footprint1',
                    '/Robot1/Twist1',
                    '/Robot1/Sensors1',
                    '/Robot1/Sensors1/Laser1',
                    '/Robot1/Sensors1/RGBD scan1',
                    '/Robot1/Sensors1/Virtual Obstacles1',
                    '/Robot1/Sensors1/Depth Cloud1',
                    '/Robot1/Sensors1/Sonars1',
                    '/Control1',
                    '/Control1/Move1',
                    '/Control1/Path1',
                    '/Planning1',
                    '/Planning1/Base1',
                    '/Planning1/Base1/Global1',
                    '/Planning1/Base1/Global1/Plan1',
                    '/Planning1/Base1/Global1/Plan1/Navfn1',
                    '/Planning1/Base1/Global1/Plan1/Global Planner1',
                    '/Planning1/Base1/Global1/Potential1',
                    '/Planning1/Base1/Global1/Costmap1',
                    '/Planning1/Base1/Global1/Voxel Grid1',
                    '/Planning1/Base1/Local1',
                    '/Planning1/Base1/Local1/Plan1',
                    '/Planning1/Base1/Local1/Plan1/Trajectory1',
                    '/Planning1/Base1/Local1/Plan1/DWA1',
                    '/Planning1/Base1/Local1/Plan1/Pal1',
                    '/Planning1/Base1/Local1/Costmap1',
                    '/Planning1/Base1/Local1/Cost1',
                    '/Planning1/Base1/Local1/Trajectories1',
                    '/Planning1/Base1/Goal1',
                    '/Localization1',
                    '/Localization1/Pose1',
                    '/Localization1/Pose Graph1',
                    '/Localization1/Localization Quality1',
                    '/Mapping1',
                    '/Mapping1/Map1',
                    '/Mapping1/SLAM Graph Edges1',
                    '/Mapping1/SLAM Graph Vertices1',
                    '/Mapping1/Trajectory1',
                    '/RGBD1',
                    '/MarkerArray1'
                    '/Marker1' ]
        dict_property_tree_widget = {'Expanded' : Expanded, 'Splitter Ratio' : 0.458522}

        dict_class_rviz_displays.update({'Porperty Tree Widget' : 470})
        dict_class_rviz_displays.update({'Tree Height' : dict_property_tree_widget})

        Panels.append(dict_class_rviz_displays)
        dict.update({'Panels' : Panels})

        rviz_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'rviz', 'auto_gen.rviz')
        with codecs.open(rviz_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(dict, outfile, default_flow_style=False)
