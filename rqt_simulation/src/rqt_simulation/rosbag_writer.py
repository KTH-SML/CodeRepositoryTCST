#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosbag
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal

def amcl_pose_cb(msg, source):
    global num_robots
    global bag
    global active
    if active:
        for i in range(0, num_robots):
            if source == '/robot' + str(i+1):
                bag.write('/robot' + str(i+1) + '/pose', msg)

def goal_cb(msg, source):
    global num_robots
    global bag
    global active
    if active:
        for i in range(0, num_robots):
            if source == '/robot' + str(i+1):
                bag.write('/robot' + str(i+1) + '/move_base/goal', msg)

def active_cb(msg):
    global active
    active = msg.data

def close_bag():
    global bag
    bag.close()


def rosbag_writer():
    global num_robots
    global bag
    global active
    active = False
    rospy.Subscriber('/logger_active', Bool, active_cb)
    bag = rosbag.Bag('/home/lukas/catin_ws/rqt_simulation.bag', 'w')
    num_robots = rospy.get_param('num_robots')
    for i in range(0, num_robots):
        rospy.Subscriber('/robot' + str(i+1) + '/pose', PoseWithCovarianceStamped, amcl_pose_cb, '/robot' + str(i+1))
        rospy.Subscriber('/robot' + str(i+1) + '/move_base/goal', MoveBaseActionGoal, goal_cb, '/robot' + str(i+1))


if __name__== '__main__':
    rospy.init_node('rosbag_writer')
    rosbag_writer()
    rospy.on_shutdown(close_bag)
    rospy.spin()
