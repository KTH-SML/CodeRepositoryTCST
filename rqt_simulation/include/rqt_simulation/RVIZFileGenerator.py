# -*- coding: utf-8 -*-
import yaml
import codecs
import os
import sys
import rospy
import rospkg

class RVIZFileGenerator:
    def __init__(self, robot_list):
        dict = {}

        self.robot_name = 'robot1'
        # Options for Displays window
        Panels = []
        dict_class_rviz_displays = {'Class' : 'rviz/Displays'}
        dict_class_rviz_displays.update({'Help Height' : 75})
        dict_class_rviz_displays.update({'Name' : 'Displays'})

        # Trees closed
        dict_property_tree_widget = {'Expanded' : '~', 'Splitter Ratio' : 0.5}

        dict_class_rviz_displays.update({'Porperty Tree Widget' : dict_property_tree_widget})
        dict_class_rviz_displays.update({'Tree Height' : 531})

        Panels.append(dict_class_rviz_displays)

        # RVIZ time
        dict_class_rviz_time = {'Class' : 'rviz/Time'}
        dict_class_rviz_time.update({'Expanded' : 'False'})
        dict_class_rviz_time.update({'Name' : 'Time'})
        dict_class_rviz_time.update({'SyncMode' : 0})
        dict_class_rviz_time.update({'SyncSource' : 'RGBD'})

        Panels.append(dict_class_rviz_time)

        # Visualization_manager for displayed topics
        dict_visualization_manager = {'Class' : ""}
        self.vis_manager_displays = []

        # Grid
        self.vis_manager_displays.append(self.add_grid())

        for i in range(0, len(robot_list)):
            self.robot_name = 'robot' + str(1+i)
            self.add_robot(robot_list[i])

        # Region of interest marker
        self.vis_manager_displays.append({  'Class' : 'rviz/MarkerArray',
                                            'Enabled' : True,
                                            'Marker Topic' : '/region_of_interest',
                                            'Name' : 'ROIMarker',
                                            'Namespaces' : {
                                                 "" : True },
                                            'Queue Size' : 100,
                                            'Value' : True })





        dict_visualization_manager.update({'Displays' : self.vis_manager_displays})

        dict_visualization_manager.update({ 'Enabled' : True,
                                            'Global Options' : {
                                              'Background Color' : '48; 48; 48',
                                              'Fixed Frame' : '/map',
                                              'Frame Rate' : 30 },
                                            'Name' : 'root',
                                            'Tools': [
                                          { 'Class' : 'rviz/Interact',
                                            'Hide Inactive Objects' : True },
                                          { 'Class' : 'rviz/MoveCamera' },
                                          { 'Class' : 'rviz/Select' },
                                          { 'Class' : 'rviz/FocusCamera' },
                                          { 'Class' : 'rviz/Measure' },
                                          { 'Class' : 'rviz/SetInitialPose',
                                            'Topic' : 'robot1/initialpose' },
                                          { 'Class' : 'rviz/SetGoal',
                                            'Topic' : 'robot1/move_base_simple/goal' },
                                          { 'Class' : 'rviz/PublishPoint',
                                            'Single click' : True,
                                            'Topic' : 'clicked_point' }],
                                        'Value' : True })


        dict.update({'Panels' : Panels, 'Visualization Manager' : dict_visualization_manager})

        dict.update({   'Window Geometry' : {
                        'Displays' : {
                          'collapsed' : False },
                        'Height' : 1028,
                        'Hide Left Dock' : False,
                        'Hide Right Dock' : False,
                        'QMainWindow State' : '000000ff00000000fd0000000300000000000001c600000399fc0200000003fb000000100044006900730070006c00610079007301000000280000029f000000d700fffffffb0000001200530065006c0065006300740069006f006e0000000166000000bb0000006100fffffffb00000008005200470042004401000002cd000000f40000001600ffffff000000010000015c000003dafc0200000002fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730000000028000003da0000006100fffffffb0000000a005600690065007700730000000028000003da000000ad00ffffff000000030000073f0000003bfc0100000001fb0000000800540069006d006501000000000000073f0000030000ffffff000005730000039900000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000',
                        'Camera': {
                          'collapsed' : False },
                        'Selection' : {
                          'collapsed' : False },
                        'Time' : {
                          'collapsed' : False },
                        'Tool Properties' : {
                          'collapsed' : False },
                        'Views' : {
                          'collapsed' : False },
                        'Width' : 1855,
                        'X' : 55,
                        'Y' : 14}})

        rviz_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'rviz', 'auto_gen.rviz')
        with codecs.open(rviz_file, 'w', encoding='utf-8') as outfile:
            yaml.safe_dump(dict, outfile, default_flow_style=False)

    def add_robot(self, robot_model):

        #Folder for robot
        dict_robot = {'Class' : 'rviz/Group'}
        robot = []

        # Folder with robot model, robot sensors and footprint
        dict_robot_folder = {'Class' : 'rviz/Group'}
        robot_folder = []

        # Robot model
        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            tf_prefix = self.robot_name
        elif robot_model == 'srd250':
            tf_prefix = ''
        robot_folder.append(self.add_robot_model(self.robot_name, tf_prefix))

        # Robot odometry
        if robot_model == 'tiago':
            odom_topic = '/' + self.robot_name + '/mobile_base_controller/odom'
            robot_folder.append(self.add_robot_odometry(odom_topic))

        # Robot footprint
        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            footprint_topic = '/' + self.robot_name + '/move_base/local_costmap/footprint'
            robot_folder.append(self.add_robot_footprint(footprint_topic))

        # Robot name marker
        label_marker_topic = '/' + self.robot_name + '/label_marker'
        robot_folder.append(self.add_marker(label_marker_topic))

        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            # Folder with robot sensors
            dict_robot_sensors = {'Class' : 'rviz/Group'}
            robot_sensors_folder = []

            # Robot laser
            laser_topic = '/' + self.robot_name + '/scan'
            robot_sensors_folder.append(self.add_robot_laser(laser_topic))

            # Robot RGBD scan
            if robot_model == 'tiago':
                RGBD_scan_topic = '/' + self.robot_name + '/rgbd_scan'
                robot_sensors_folder.append(self.add_robot_RGBD_scan(RGBD_scan_topic))

            # Robot depth cloud
            if robot_model == 'tiago':
                depth_cloud_topic = '/' + self.robot_name + '/xtion/depth_registered/points'
            elif robot_model == 'turtlebot':
                depth_cloud_topic = '/' + self.robot_name + '/camera/depth/points'
            robot_sensors_folder.append(self.add_robot_depth_cloud(depth_cloud_topic))

            # Sonar cloud
            if robot_model == 'tiago':
                sonar_topic = '/' + self.robot_name + '/sonar_cloud'
                robot_sensors_folder.append(self.add_robot_sonar_cloud(sonar_topic))

            dict_robot_sensors.update({'Displays' : robot_sensors_folder})
            dict_robot_sensors.update({'Enabled' : True, 'Name' : 'Sensors'})
            robot_folder.append(dict_robot_sensors)

        dict_robot_folder.update({'Displays' : robot_folder})
        dict_robot_folder.update({'Enabled' : True, 'Name' : 'Robot'})

        robot.append(dict_robot_folder)

        # Planning folder
        dict_planning_folder = {'Class' : 'rviz/Group'}
        planning_folder = []

        # Base folder
        dict_base_folder = {'Class' : 'rviz/Group'}
        base_folder = []

        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            # Global planner folder
            dict_global_folder = {'Class' : 'rviz/Group'}
            global_folder = []

            # Global Plan folder
            dict_global_plan_folder = {'Class' : 'rviz/Group'}
            global_plan_folder = []

            # Global NavfnROS planner
            if robot_model == 'tiago':
                Navfn_topic = '/' + self.robot_name + '/move_base/NavfnROS/plan'
            elif robot_model == 'turtlebot':
                Navfn_topic = '/' + self.robot_name + '/move_base/GlobalPlanner/plan'
            global_plan_folder.append(self.add_path(Navfn_topic, 'Navfn'))

            # Global EBandPlannerROS planner
            if robot_model == 'tiago':
                global_plan_topic = '/' + self.robot_name + '/move_base/EBandPlannerROS/global_plan'
            elif robot_model == 'turtlebot':
                global_plan_topic = '/' + self.robot_name + '/move_base/DWAPlannerROS/global_plan'
            global_plan_folder.append(self.add_path(global_plan_topic, 'Global'))

            dict_global_plan_folder.update({'Displays' : global_plan_folder})
            dict_global_plan_folder.update({'Enabled' : True, 'Name' : 'Plan'})

            global_folder.append(dict_global_plan_folder)

            # Global potential costmap
            if robot_model == 'tiago':
                global_potential_topic = '/' + self.robot_name + '/move_base/global_costmap/costmap'
            elif robot_model == 'turtlebot':
                global_potential_topic = '/' + self.robot_name + '/move_base/GlobalPlanner/potential'
            global_folder.append(self.add_costmap(global_potential_topic))

            # Global costmap
            global_costmap_topic = '/' + self.robot_name + '/move_base/global_costmap/costmap'
            global_folder.append(self.add_costmap(global_costmap_topic))

            dict_global_folder.update({'Displays' : global_folder})
            dict_global_folder.update({'Enabled' : True, 'Name' : 'Global'})

            base_folder.append(dict_global_folder)

            # Local planner folder
            dict_local_folder = {'Class' : 'rviz/Group'}
            local_folder = []

            # Local Plan folder
            dict_local_plan_folder = {'Class' : 'rviz/Group'}
            local_plan_folder = []

            # Local EBandPlannerROS
            if robot_model == 'tiago':
                local_eband_topic = '/' + self.robot_name + '/move_base/EBandPlannerROS/local_plan'
                local_plan_folder.append(self.add_path(local_eband_topic, 'EBand'))

            # Local DWAPlannerROS
            dwa_plan_topic = '/' + self.robot_name + '/move_base/DWAPlannerROS/local_plan'
            local_plan_folder.append(self.add_path(dwa_plan_topic, 'DWA'))

            # Local PalPlannerROS
            if robot_model == 'tiago':
                local_pal_topic = '/' + self.robot_name + '/move_base/PalPlannerROS/local_plan'
                local_plan_folder.append(self.add_path(local_pal_topic, 'Pal'))

            dict_local_plan_folder.update({'Displays' : local_plan_folder})
            dict_local_plan_folder.update({'Enabled' : True, 'Name' : 'Plan'})

            local_folder.append(dict_local_plan_folder)

            # Local costmap
            local_costmap_topic = '/' + self.robot_name + '/move_base/local_costmap/costmap'
            local_folder.append(self.add_costmap(local_costmap_topic))

            # Local costcloud
            if robot_model == 'tiago':
                local_costcloud_topic = '/' + self.robot_name + '/move_base/TrajectoryPlannerROS/cost_cloud'
            elif robot_model == 'turtlebot':
                local_costcloud_topic = '/' + self.robot_name + '/move_base/DWAPlannerROS/cost_cloud'
            local_folder.append(self.add_costcloud(local_costcloud_topic))

            dict_local_folder.update({'Displays' : local_folder})
            dict_local_folder.update({'Enabled' : True, 'Name' : 'Local'})

            base_folder.append(dict_local_folder)

        # Goal marker
        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            goal_topic = '/' + self.robot_name + '/move_base/current_goal'
        elif robot_model == 'srd250':
            goal_topic = '/' + self.robot_name + '/command/pose'
        base_folder.append(self.add_goal_marker(self.robot_name, goal_topic))

        dict_base_folder.update({'Displays' : base_folder})
        dict_base_folder.update({'Enabled' : True, 'Name' : 'Base'})


        planning_folder.append(dict_base_folder)
        dict_planning_folder.update({'Displays' : planning_folder})
        dict_planning_folder.update({'Enabled' : True, 'Name' : 'Planning'})

        robot.append(dict_planning_folder)

        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            dict_localization_folder = {'Class' : 'rviz/Group'}
            localization_folder = []

            # Localization
            localization_folder.append(self.add_particlecloud(self.robot_name))

            dict_localization_folder.update({'Displays' : localization_folder})
            dict_localization_folder.update({'Enabled' : True, 'Name' : 'Localization'})

            robot.append(dict_localization_folder)

        dict_mapping_folder = {'Class' : 'rviz/Group'}
        mapping_folder = []

        # Robot map
        mapping_folder.append(self.add_map(self.robot_name))

        # Trajectory
        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            mapping_folder.append(self.add_slam_trajectory(self.robot_name))

        dict_mapping_folder.update({'Displays' : mapping_folder})
        dict_mapping_folder.update({'Enabled' : True, 'Name' : 'Mapping'})

        robot.append(dict_mapping_folder)

        if (robot_model == 'tiago') or (robot_model == 'turtlebot'):
            # Camera
            if robot_model == 'tiago':
                image_topic = '/' + self.robot_name + '/xtion/rgb/image_raw'
            elif robot_model == 'turtlebot':
                image_topic = '/' + self.robot_name + '/camera/rgb/image_raw'
            robot.append(self.add_camera(image_topic))

        dict_robot.update({'Displays' : robot})
        dict_robot.update({'Enabled' : True, 'Name' : self.robot_name})

        self.vis_manager_displays.append(dict_robot)

    def add_grid(self):
        dict_grid = {   'Alpha' : 0.5,
                        'Cell Size' : 1,
                        'Class' : 'rviz/Grid',
                        'Color' : '160; 160; 164',
                        'Enabled' : 'True',
                        'Line Style' : {
                            'Line Width' : 0.03,
                            'Values': 'Lines'},
                         'Name' : 'Grid',
                         'Normal Cell Count' : 0,
                         'Offset' : {
                            'X' : 0,
                            'Y' : 0,
                            'Z' : 0},
                         'Plane' : 'XY',
                         'Plane Cell Count' : 10,
                         'Reference Frame': '<Fixed Frame>',
                         'Value' : True }

        return dict_grid

    def add_robot_model(self, robot_name, tf_prefix):
        robot_model =     { 'Alpha': 1,
                            'Class' : 'rviz/RobotModel',
                            'Collision Enabled' : False,
                            'Enabled': True,
                            'Name' : 'RobotModel',
                            'Robot Description' : robot_name + '/robot_description',
                            'TF Prefix' : tf_prefix,
                            'Update Interval' : 0,
                            'Value' : True,
                            'Visual Enabled' : True }

        return robot_model

    def add_robot_odometry(self, topic):
        robot_odometry =      { '6DOF' : False,
                                'Alpha' : 1,
                                'Axis' : False,
                                'Class' : 'rviz_plugin_covariance/Odometry',
                                'Color' : '204; 51; 204',
                                'Enabled' : False,
                                'Name' : 'Odometry',
                                'Orientation' : False,
                                'Position' : False,
                                'Scale' : 1,
                                'Topic' : topic,
                                'Unreliable' : False,
                                'Value' : False }

        return robot_odometry

    def add_robot_footprint(self, topic):
        robot_footprint =     { 'Alpha' : 1,
                                'Class' : 'rviz/Polygon',
                                'Color' : '255; 255; 0',
                                'Enabled' : True,
                                'Name' : 'Footprint',
                                'Topic' : topic,
                                'Unreliable' : False,
                                'Value' : True}

        return robot_footprint

    def add_marker(self, topic):
        marker =  { 'Class' : 'rviz/Marker',
                    'Enabled' : True,
                    'Marker Topic' : topic,
                    'Name' : 'ROIMarker',
                    'Namespaces' : {
                         "" : True },
                    'Queue Size' : 100,
                    'Value' : True }

        return marker

    def add_robot_laser(self, topic):
        robot_laser = { 'Alpha' : 0.5,
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
                        'Topic' : topic,
                        'Unreliable' : False,
                        'Use Fixed Frame' : True,
                        'Use rainbow' : True,
                        'Value' : True }

        return robot_laser

    def add_robot_RGBD_scan(self, topic):
        robot_RGBD_scan = { 'Alpha' : 1,
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
                            'Topic' : topic,
                            'Unreliable' : False,
                            'Use Fixed Frame' : True,
                            'Use rainbow' : True,
                            'Value' : True }

        return robot_RGBD_scan

    def add_robot_depth_cloud(self, topic):
        robot_depth_cloud = {   'Alpha' : 1,
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
                                'Topic' : topic,
                                'Unreliable' : False,
                                'Use Fixed Frame' : True,
                                'Use rainbow' : True,
                                'Value' : False }

        return robot_depth_cloud

    def add_robot_sonar_cloud(self, topic):
        robot_sonar_cloud = {   'Alpha' : 1,
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
                                'Topic' : topic,
                                'Unreliable' : False,
                                'Use Fixed Frame' : True,
                                'Use rainbow' : True,
                                'Value' : True }

        return robot_sonar_cloud

    def add_path(self, topic, planner_type):
        path = {'Alpha' : 1,
                'Buffer Length' : 1,
                'Class' : 'rviz/Path',
                'Color' : '0; 0; 255',
                'Enabled' : True,
                'Head Diameter' : 0.3,
                'Head Length' : 0.2,
                'Length' : 0.3,
                'Line Style' : 'Lines',
                'Line Width' : 0.03,
                'Name' : planner_type,
                'Offset' : {
                  'X' : 0,
                  'Y' : 0,
                  'Z' : 0 },
                'Pose Color' : '255; 85; 255',
                'Pose Style' : 'None',
                'Radius' : 0.03,
                'Shaft Diameter' : 0.1,
                'Shaft Length' : 0.1,
                'Topic' : topic,
                'Unreliable' : False,
                'Value' : True }

        return path

    def add_costmap(self, topic):
        costmap = { 'Alpha' : 0.7,
                    'Class' : 'rviz/Map',
                    'Color Scheme' : 'costmap',
                    'Draw Behind' : True,
                    'Enabled' : True,
                    'Name' : 'Costmap',
                    'Topic' : topic,
                    'Unreliable' : False,
                    'Value' : True }

        return costmap

    def add_costcloud(self, topic):
        costcloud = {   'Alpha' : 0.2,
                        'Autocompute Intensity Bounds' : True,
                        'Autocompute Value Bounds' : {
                          'Max Value' : 0,
                          'Min Value' : 0,
                          'Value' : True },
                        'Axis' : 'Z',
                        'Channel Name' : 'total_cost',
                        'Class' : 'rviz/PointCloud2',
                        'Color' : '255; 255; 255',
                        'Color Transformer' : 'Intensity',
                        'Decay Time' : 0,
                        'Enabled' : True,
                        'Invert Rainbow' : False,
                        'Max Color' : '255; 255; 255',
                        'Max Intensity' : 7.06,
                        'Min Color' : '0; 0; 0',
                        'Min Intensity' : 0.56,
                        'Name' : 'Cost',
                        'Position Transformer': 'XYZ',
                        'Queue Size' : 10,
                        'Selectable' : True,
                        'Size (Pixels)' : 3,
                        'Size (m)' : 0.07,
                        'Style' : 'Flat Squares',
                        'Topic' : topic,
                        'Unreliable' : False,
                        'Use Fixed Frame' : True,
                        'Use rainbow' : True,
                        'Value' : True }

        return costcloud

    def add_goal_marker(self, robot_name, topic):
        goal_marker = { 'Alpha' : 1,
                        'Axes Length' : 1,
                        'Axes Radius' : 0.1,
                        'Class' : 'rviz/Pose',
                        'Color' : '255; 25; 0',
                        'Enabled' : True,
                        'Head Length' : 0.3,
                        'Head Radius' : 0.1,
                        'Name' : 'Goal',
                        'Shaft Length' : 1,
                        'Shaft Radius' : 0.05,
                        'Shape' : 'Arrow',
                        'Topic' : topic,
                        'Unreliable' : False,
                        'Value' : True }

        return goal_marker

    def add_particlecloud(self, robot_name):
        particlecloud = {   'Arrow Length' : 0.3,
                            'Class' : 'rviz/PoseArray',
                            'Color' : '255; 25; 0',
                            'Enabled' : True,
                            'Name' : 'Pose',
                            'Topic' : '/' + robot_name + '/particlecloud',
                            'Unreliable' : 'False',
                            'Value' : True }

        return particlecloud

    def add_map(self, robot_name):
        map = { 'Alpha' : 0.2,
                'Class' : 'rviz/Map',
                'Color Scheme' : 'map',
                'Draw Behind' : True,
                'Enabled' : True,
                'Name' : 'Map',
                'Topic' : '/' + robot_name + '/map',
                'Unreliable' : False,
                'Value' : True }

        return map

    def add_slam_trajectory(self, robot_name):
        trajectory = {  'Arrow Length' : 0.3,
                        'Class' : 'rviz/PoseArray',
                        'Color' : '255; 25; 0',
                        'Enabled' : 'True',
                        'Name' : 'Trajectory',
                        'Topic' : '/' + robot_name + '/slam_trajectory',
                        'Unreliable' : False,
                        'Value' : True }

        return trajectory

    def add_camera(self, topic):
        camera = {     'Class' : 'rviz/Camera',
                       'Enabled' : True,
                       'Image Rendering' : 'background and overlay',
                       'Image Topic' : topic,
                       'Name' : 'Camera',
                       'Overlay Alpha' : 0.5,
                       'Queue Size' : 2,
                       'Transport Hint' : 'raw',
                       'Unreliable' : False,
                       'Value ': True,
                       'Zoom Factor' : 1 }
        return camera
