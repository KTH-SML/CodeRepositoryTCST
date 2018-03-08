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
                    '/Robot1/Footprint1',
                    '/Robot1/Dynamic Footprint1',
                    '/Robot1/Twist1',
                    '/Robot1/Sensors1',
                    '/Robot1/Sensors1/Laser1',
                    '/Robot1/Sensors1/RGBD scan1',
                    '/Planning1',
                    '/Planning1/Base1',
                    '/Planning1/Base1/Global1',
                    '/Planning1/Base1/Global1/Plan1',
                    '/Planning1/Base1/Global1/Plan1/Navfn1',
                    '/Planning1/Base1/Global1/Plan1/Global Planner1',
                    '/Planning1/Base1/Global1/Potential1',
                    '/Planning1/Base1/Global1/Costmap1',
                    '/Planning1/Base1/Global1/Costmap1/Status1',
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
                    '/Mapping1',
                    '/Mapping1/Map1',
                    '/Mapping1/Trajectory1',
                    '/RGBD1',
                    '/MarkerArray1']

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
        dict_class_rviz_time.update({'Expanded' : 'False'})
        dict_class_rviz_time.update({'Name' : 'Time'})
        dict_class_rviz_time.update({'SyncMode' : 0})
        dict_class_rviz_time.update({'SyncSource' : 'RGBD'})
        Panels.append(dict_class_rviz_time)

        #Visualization_manager = {}
        dict_class = {'Class' : ""}
        vis_manager_displays = []
        dict_displays = {'Alpha' : 0.5, 'Cell Size' : 1, 'Class' : 'rviz/Grid', 'Color' : '160; 160; 164',
                         'Enabled' : 'True', 'Line Style' : {'Line Width' : 0.03, 'Values': 'Lines'},
                         'Name' : 'Grid', 'Normal Cell Count' : 0, 'Offset' : {'X' : 0, 'Y' : 0, 'Z' : 0},
                         'Plane' : 'XY', 'Plane Cell Count' : 10, 'Reference Frame': '<Fixed Frame>',
                         'Value' : 'True'}

        vis_manager_displays.append(dict_displays)

        dict_class_rviz_group = {'Class' : 'rviz/Group'}
        displays_group = [{ 'Alpha': 1,
                            'Class' : 'rviz/RobotModel',
                            'Collision Enabled' : False,
                            'Enabled': True,
                            'Links' :
                              {'All Links Enabled' : True,
                               'Expand Joint Details' : False,
                               'Expand Link Details' : False,
                               'Expand Tree' : False,
                              'Link Tree Style' : 'Links in Alphabetic Order',
                              'arm_1_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail': False,
                                 'Value' : True},
                              'arm_2_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail': False,
                                 'Value' : True},
                              'arm_3_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail': False,
                                 'Value' : True},
                              'arm_4_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail': False,
                                 'Value' : True},
                              'arm_5_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail': False,
                                 'Value' : True},
                              'arm_6_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail': False,
                                 'Value' : True},
                              'arm_7_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail': False,
                                 'Value' : True},
                              'arm_tool_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'base_antenna_left_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'base_antenna_right_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'base_cover_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'base_footprint' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False},
                              'base_imu_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False},
                              'base_laser_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'base_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'base_sonar_01_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'base_sonar_02_link' :
                                {'Alpha' : 1,
                                   'Show Axes' : False,
                                   'Show Trail' : False,
                                   'Value' : True},
                              'base_sonar_03_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_back_left_1_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_back_left_2_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_back_right_1_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_back_right_2_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_front_left_1_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_front_left_2_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_front_right_1_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'caster_front_right_2_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'gripper_grasping_frame' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False},
                              'gripper_left_finger_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'gripper_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'gripper_right_finger_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'head_1_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'head_2_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'torso_fixed_column_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'torso_fixed_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'torso_lift_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'wheel_left_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'wheel_right_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False,
                                 'Value' : True},
                              'xtion_depth_frame' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False},
                              'xtion_depth_optical_frame' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False},
                              'xtion_link' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False},
                              'xtion_optical_frame' :
                                {'Alpha' : 1,
                                   'Show Axes' : False,
                                   'Show Trail' : False},
                              'xtion_rgb_frame' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False},
                              'xtion_rgb_optical_frame' :
                                {'Alpha' : 1,
                                 'Show Axes' : False,
                                 'Show Trail' : False}
                                 },
                           'Name' : 'RobotModel',
                           'Robot Description' : 'robot1/robot_description',
                           'TF Prefix' : 'robot1',
                           'Update Interval' : 0,
                           'Value' : True,
                           'Visual Enabled' : True
                                                    }]

        displays_group.append({ 'Class' : 'rviz/TF',
                                'Enabled' : False,
                                'Frame Timeout' : 15,
                                'Frames' :
                                  {'All Enabled' : True},
                                'Marker Scale' : 0.1,
                                'Name' : 'TF',
                                'Show Arrows' : True,
                                'Show Axes' : True,
                                'Show Names' : True,
                                'Tree' : {},
                                'Update Interval' : 0,
                                'Value' : False})

        displays_group.append({ '6DOF' : False,
                                'Alpha' : 1,
                                'Axis' : False,
                                'Class' : 'rviz_plugin_covariance/Odometry',
                                'Color' : '204; 51; 204',
                                'Enabled' : False,
                                'Name' : 'Odometry',
                                'Orientation' : False,
                                'Position' : False,
                                'Scale' : 1,
                                'Topic' : '/tiago1/mobile_base_controller/odom',
                                'Unreliable' : False,
                                'Value' : False})

        displays_group.append({ 'Alpha' : 1,
                                'Class' : 'rviz/Polygon',
                                'Color' : '255; 255; 0',
                                'Enabled' : True,
                                'Name' : 'Footprint',
                                'Topic' : '/robot1/move_base/local_costmap/footprint',
                                'Unreliable' : False,
                                'Value' : True})

        displays_group.append({ 'Class' : 'rviz/Group',
                                'Displays' : [
                                  { 'Alpha' : 0.5,
                                    'Autocompute Intensity Bounds' : True,
                                    'Autocompute Value Bounds' :
                                    { 'Max Value' : 10,
                                      'Min Value' : -10,
                                      'Value' : True },
                                    'Axis' : 'Z',
                                    'Channel Name' : 'intensity',
                                    'Class' : 'rviz/LaserScan',
                                    'Color' : '0; 0; 255',
                                    'Color Transformer' : 'FlatColor',
                                    'Decay Time' : 0,
                                    'Enabled' : True,
                                    'Invert Rainbow' : False,
                                    'Max Color' : '255; 255; 255',
                                    'Max Intensity' : 0,
                                    'Min Color' : '0; 0; 0',
                                    'Min Intensity' : 0,
                                    'Name' : 'Laser',
                                    'Position Transformer': 'XYZ',
                                    'Queue Size' : 10,
                                    'Selectable' : True,
                                    'Size (Pixels)' : 3,
                                    'Size (m)' : 0.01,
                                    'Style' : 'Points',
                                    'Topic' : '/robot1/scan',
                                    'Unreliable' : False,
                                    'Use Fixed Frame' : True,
                                    'Use rainbow' : True,
                                    'Value' : True},

                                  { 'Alpha' : 1,
                                    'Autocompute Intensity Bounds' : True,
                                    'Autocompute Value Bounds' :
                                    { 'Max Value' : 10,
                                      'Min Value' : -10,
                                      'Value' : True },
                                    'Axis' : 'Z',
                                    'Channel Name' : 'intensity',
                                    'Class' : 'rviz/LaserScan',
                                    'Color' : '255; 0; 127',
                                    'Color Transformer' : 'FlatColor',
                                    'Decay Time' : 0,
                                    'Enabled' : True,
                                    'Invert Rainbow' : False,
                                    'Max Color' : '255; 255; 255',
                                    'Max Intensity' : 4096,
                                    'Min Color' : '0; 0; 0',
                                    'Min Intensity' : 0,
                                    'Name' : 'RGBD scan',
                                    'Position Transformer' : 'XYZ',
                                    'Queue Size' : 10,
                                    'Selectable' : True,
                                    'Size (Pixels)' : 3,
                                    'Size (m)' : 0.05,
                                    'Style' : 'Flat Squares',
                                    'Topic' : '/robot1/rgbd_scan',
                                    'Unreliable' : False,
                                    'Use Fixed Frame' : True,
                                    'Use rainbow' : True,
                                    'Value' : True },

                                  { 'Alpha' : 1,
                                    'Autocompute Intensity Bounds': True,
                                    'Autocompute Value Bounds':
                                    {  'Max Value' : 10,
                                       'Min Value' : -10,
                                       'Value' : True},
                                    'Axis' : 'Z',
                                    'Channel Name' : 'intensity',
                                    'Class' : 'rviz/PointCloud2',
                                    'Color' : '255; 255; 255',
                                    'Color Transformer' : 'Intensity',
                                    'Decay Time' : 0,
                                    'Enabled' : False,
                                    'Invert Rainbow' : False,
                                    'Max Color' : '255; 255; 255',
                                    'Max Intensity' : 4096,
                                    'Min Color' : '0; 0; 0',
                                    'Min Intensity' : 0,
                                    'Name' : 'Depth Cloud',
                                    'Position Transformer' : 'XYZ',
                                    'Queue Size' : 1,
                                    'Selectable' : True,
                                    'Size (Pixels)' : 3,
                                    'Size (m)' : 0.01,
                                    'Style' : 'Points',
                                    'Topic' : '/robot1/xtion/depth_registered/points',
                                    'Unreliable' : False,
                                    'Use Fixed Frame' : True,
                                    'Use rainbow' : True,
                                    'Value' : False },

                                  { 'Alpha' : 1,
                                    'Autocompute Intensity Bounds': True,
                                    'Autocompute Value Bounds':
                                    {  'Max Value' : -1.27299,
                                       'Min Value' : -1.67406,
                                       'Value': True},
                                    'Axis' : 'X',
                                    'Channel Name' : 'intensity',
                                    'Class' : 'rviz/PointCloud2',
                                    'Color' : '255; 255; 255',
                                    'Color Transformer': 'FlatColor',
                                    'Decay Time' : 0,
                                    'Enabled' : True,
                                    'Invert Rainbow' : False,
                                    'Max Color' : '255; 255; 255',
                                    'Max Intensity' : 4096,
                                    'Min Color' : '0; 0; 0',
                                    'Min Intensity' : 0,
                                    'Name' : 'Sonar Cloud',
                                    'Position Transformer' : 'XYZ',
                                    'Queue Size' : 10,
                                    'Selectable' : True,
                                    'Size (Pixels)' : 10,
                                    'Size (m)' : 0.01,
                                    'Style' : 'Points',
                                    'Topic' : '/sonar_cloud',
                                    'Unreliable' : False,
                                    'Use Fixed Frame' : True,
                                    'Use rainbow' : True,
                                    'Value' : True }],

                                'Enabled' : True,
                                'Name' : 'Robot' })


        dict_class_rviz_group.update({'Displays' : displays_group})

        vis_manager_displays.append(dict_class_rviz_group)
        dict_class.update({'Displays' : vis_manager_displays})

        #Visualization_manager.update({'Visualization Manager' : dict_class})



        dict.update({'Panels' : Panels, 'Visualization Manager' : dict_class})

        rviz_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'rviz', 'auto_gen.rviz')
        with codecs.open(rviz_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(dict, outfile, default_flow_style=False)
