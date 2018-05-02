#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import yaml
import rospy
import rosbag
import rospkg
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal

class RosbagWriterNode(object):
    def __init__(self):
        self.node_name = "Rosbag Writer"

        env_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
        stream = file(env_file, 'r')
        data = yaml.load(stream)
        stream.close()

        self.robot_names = data['Tasks'].keys()
        print(self.robot_names)

        self.active = False
        rospy.Subscriber('/logger_active', Bool, self.active_cb)

        self.bag = rosbag.Bag('/home/sml/tiago_public_ws/rqt_simulation.bag', 'w')

        self.pose_sub_list = []
        self.goal_sub_list = []

        for i in range(0, len(self.robot_names)):
            self.pose_sub_list.append(rospy.Subscriber('/' + self.robot_names[i] + '/pose_gui', PoseWithCovarianceStamped, self.pose_cb, '/' + self.robot_names[i]))
            self.goal_sub_list.append(rospy.Subscriber('/' + self.robot_names[i] + '/move_base/goal', MoveBaseActionGoal, self.goal_cb, '/' + self.robot_names[i]))

    def active_cb(self, msg):
        self.active = msg.data

    def goal_cb(self, msg, source):
        if self.active:
            for i in range(0, len(self.robot_names)):
                self.bag.write('/' + self.robot_names[i] + '/move_base/goal', msg)

    def pose_cb(self, msg, source):
        if self.active:
            for i in range(0, len(self.robot_names)):
                self.bag.write('/' + self.robot_names[i] + '/pose', msg)

    def close_bag(self):
        self.bag.close()

if __name__== '__main__':
    rospy.init_node('rosbag_writer', anonymous=False)
    rosbag_writer_node = RosbagWriterNode()
    rospy.on_shutdown(rosbag_writer_node.close_bag)
    rospy.spin()
