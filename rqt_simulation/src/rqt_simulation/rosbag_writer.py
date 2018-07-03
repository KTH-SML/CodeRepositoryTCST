#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import yaml
import rospy
import rosbag
import rospkg
import datetime
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from rqt_simulation_msgs.msg import Sense, TemporaryTask

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

        #self.bag = rosbag.Bag(os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'rqt_simulation.bag'), 'w')

        self.pose_sub_list = []
        self.goal_sub_list = []
        self.temp_task_sub_list = []
        self.file_list = []
        self.open_files = []
        self.task_files = []
        self.task_open_files = []

        now = datetime.datetime.now()

        for i in range(0, len(self.robot_names)):
            self.file_list.append(os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'logging', self.robot_names[i] + '_trajectory_' + now.strftime("%Y-%m-%d-%H-%M") + '.txt' ))
            self.open_files.append(open(self.file_list[i], "w+"))
            self.task_files.append(os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'logging', self.robot_names[i] + '_temp_task' + now.strftime("%Y-%m-%d-%H-%M") + '.txt' ))
            self.task_open_files.append(open(self.task_files[i], "w+"))
            #self.open_files[i].write("time \t pos_x \t pos_y \t pos_z \t ori_w \t ori_x \t ori_y \t ori_z \t ori_z \n")
            self.pose_sub_list.append(rospy.Subscriber('/' + self.robot_names[i] + '/pose_gui', PoseWithCovarianceStamped, self.pose_cb, self.robot_names[i]))
            self.goal_sub_list.append(rospy.Subscriber('/' + self.robot_names[i] + '/move_base/goal', MoveBaseActionGoal, self.goal_cb, '/' + self.robot_names[i]))
            self.temp_task_sub_list.append(rospy.Subscriber('/' + self.robot_names[i] + '/temporary_task', TemporaryTask, self.task_cb, self.robot_names[i]))

        #self.env_sub = rospy.Subscriber('/environment', Sense, self.env_cb)

    def active_cb(self, msg):
        '''
        if self.active:
            if msg.data == False:
                #self.bag.close()
                for i in range(0, len(self.open_files)):
                    self.open_files[i].close()
                    self.task_open_files.close()
        '''
        self.active = msg.data


    def goal_cb(self, msg, source):
        if self.active:
            pose_msg = PoseStamped()
            pose_msg.header = msg.goal.target_pose.header
            pose_msg.pose = msg.goal.target_pose.pose
            #self.bag.write(source + '/target_pose', pose_msg)

    def pose_cb(self, msg, source):
        if self.active:
            time = (msg.header.stamp).to_sec()
            index = self.robot_names.index(source)
            self.open_files[index].write("%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f\n" % (time, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z))
            #self.bag.write(source + '/pose', msg)

    def task_cb(self, msg, source):
        if self.active:
            time = (msg.header.stamp).to_sec()
            index = self.robot_names.index(source)
            string = []
            for i in range(0, len(msg.task)):
                string.append(msg.task[i].data)
            self.task_open_files[index].write("%f \t %f \t %s\n" % (time, float(msg.T_des.data), string))
            #self.bag.write(source + '/temporary_task', msg)

    def close_files(self):
        for i in range(0, len(self.open_files)):
            self.open_files[i].close()
            self.task_open_files.close()

if __name__== '__main__':
    rospy.init_node('rosbag_writer', anonymous=False)
    rosbag_writer_node = RosbagWriterNode()
    rospy.on_shutdown(rosbag_writer_node.close_files)
    rospy.spin()
