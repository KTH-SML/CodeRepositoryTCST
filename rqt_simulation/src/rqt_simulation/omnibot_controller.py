#!/usr/bin/env python

import roslib
import numpy
import Queue
import rospy
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal


class OmnibotController(object):
    def __init__(self):
        self.node_name = "Omnibot Controller"
        self.current_pose = PoseWithCovarianceStamped()
        self.current_goal = PoseStamped()

        #------------
        # Subscribers
        #------------
        # Get current pose
        self.sub_pose = rospy.Subscriber('ground_truth/pose_with_covariance', PoseWithCovarianceStamped, self.PoseCallback)
        # Get current goal
        self.sub_goal = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.goal_callback)

        #-----------
        # Publishers
        #-----------
        self.pub_vel_cmd = rospy.Publisher('cmdvel', Twist, queue_size = 1)

    def PoseCallback(self, msg):
        self.current_pose.pose.pose = msg.pose.pose
        position = [self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z]
        goal = [self.current_goal.pose.position.x, self.current_goal.pose.position.y, self.current_goal.pose.position.z]
        V_des = compute_V_des(position, goal, Vmax)

    def goal_callback(self, msg):
        self.current_goal.pose = msg.goal.target_pose.pose



