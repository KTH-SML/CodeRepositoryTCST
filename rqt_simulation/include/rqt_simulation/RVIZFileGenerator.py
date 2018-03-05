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

        Expanded_displays = ['/Global Options1',
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

        dict_property_tree_widget = {'Expanded' : Expanded_displays, 'Splitter Ratio' : 0.458522}

        dict_class_rviz_displays.update({'Porperty Tree Widget' : 470})
        dict_class_rviz_displays.update({'Tree Height' : dict_property_tree_widget})

        Panels.append(dict_class_rviz_displays)

        dict_class_rviz_selection = {'Class' : 'rviz/Selection'}
        dict_class_rviz_selection.update({'Name' : 'Selection'})
        Panels.append(dict_class_rviz_selection)

        dict_class_rviz_tool = {'Class' : 'rviz/Tool Properties'}
        Expanded_tools = ['/2D Pose Estimate1', '/2D Nav Goal1', '/Publish Point1']
        dict_class_rviz_tool.update({'Expanded' : Expanded_tools})
        dict_class_rviz_tool.update({'Name' : 'Tool Properties'})
        dict_class_rviz_tool.update({'Splitter Ratio' : 0.600791})
        Panels.append(dict_class_rviz_tool)

        dict_class_rviz_views = {'Class' : 'rviz/Views'}
        dict_class_rviz_views.update({'Expanded' : ['/Current View1']})
        dict_class_rviz_views.update({'Name' : 'Views'})
        dict_class_rviz_views.update({'Splitter Ratio' : 0.5})
        Panels.append(dict_class_rviz_views)

        dict_class_rviz_time = {'Class' : 'rviz/Time'}
        dict_class_rviz_time.update({'Expanded' : 'false'})
        dict_class_rviz_time.update({'Name' : 'Time'})
        dict_class_rviz_time.update({'SyncMode' : 0})
        dict_class_rviz_time.update({'SyncSource' : 'RGBD'})
        Panels.append(dict_class_rviz_time)

        #Visualization_manager = {}
        dict_class = {'Class' : ""}
        vis_manager_displays = []
        dict_displays = {'Alpha' : 0.5, 'Cell Size' : 1, 'Class' : 'rviz/Grid', 'Color' : '160; 160; 164',
                         'Enabled' : 'true', 'Line Style' : {'Line Width' : 0.03, 'Values': 'Lines'},
                         'Name' : 'Grid', 'Normal Cell Count' : 0, 'Offset' : {'X' : 0, 'Y' : 0, 'Z' : 0},
                         'Plane' : 'XY', 'Plane Cell Count' : 10, 'Reference Frame': '<Fixed Frame>',
                         'Value' : 'true'}

        dict_class_rviz_group = {'Class' : 'rviz/Group'}
        displays_group = [{ 'Alpha': 1,
                            'Class' :' rviz/RobotModel',
                            'Collision Enabled' : 'false',
                            'Enabled': 'true',
                            'Links' :
                              {'All Links Enabled' : 'true',
                               'Expand Joint Details' : 'false',
                               'Expand Link Details' : 'false',
                               'Expand Tree' : 'false',
                              'Link Tree Style' : 'Links in Alphabetic Order',
                              'arm_1_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail': 'false',
                                 'Value' : 'true'},
                              'arm_2_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail': 'false',
                                 'Value' : 'true'},
                              'arm_3_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail': 'false',
                                 'Value' : 'true'},
                              'arm_4_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail': 'false',
                                 'Value' : 'true'},
                              'arm_5_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail': 'false',
                                 'Value' : 'true'},
                              'arm_6_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail': 'false',
                                 'Value' : 'true'},
                              'arm_7_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail': 'false',
                                 'Value' : 'true'},
                              'arm_tool_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : 'false',
                                 'Show Trail' : 'false',
                                 'Value' : 'true'}}}]

        vis_manager_displays.append(dict_displays)
        dict_class.update({'Displays' : vis_manager_displays})

        #Visualization_manager.update({'Visualization Manager' : dict_class})



        dict.update({'Panels' : Panels, 'Visualization Manager' : dict_class})

        rviz_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'rviz', 'auto_gen.rviz')
        with codecs.open(rviz_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(dict, outfile, default_flow_style=False)
