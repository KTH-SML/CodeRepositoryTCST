#!/usr/bin/env python

import roslib
import numpy
import Queue
import rospy
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal


class RVOController(object):
    def __init__(self):
        self.node_name = "RVO Controller"
        self.current_pose1 = PoseWithCovarianceStamped()
        self.current_pose2 = PoseWithCovarianceStamped()
        self.current_twist1 = Twist()
        self.current_twist2 = Twist()
        self.current_position_list = [[self.current_pose1.pose.position.x, self.current_pose1.pose.position.y, self.current_pose1.pose.position.z], [self.current_pose2.pose.position.x, self.current_pose2.pose.position.y, self.current_pose2.pose.position.z]]
        self.current_vel_list = [[self.current_twist1.linear.x, self.current_twist1.linear.y, self.current_twist1.linear.z], [self.current_twist2.linear.x, self.current_twist2.linear.y, self.current_twist2.linear.z]]

        #------------
        # Subscribers
        #------------
        # Get current pose
        #self.sub_pose = rospy.Subscriber('ground_truth/pose_with_covariance', PoseWithCovarianceStamped, self.PoseCallback)
        self.sub_pose1 = rospy.Subscriber('/robot1/amcl_pose', PoseWithCovarianceStamped, self.PoseCallback1)
        self.sub_pose2 = rospy.Subscriber('/robot2/amcl_pose', PoseWithCovarianceStamped, self.PoseCallback2)
        # Get desired velocity
        self.sub_vel1 = rospy.Subscriber('/robot1/mobile_base/commands/velocity', Twist, self.vel_callback1)
        self.sub_vel2 = rospy.Subscriber('/robot2/mobile_base/commands/velocity', Twist, self.vel_callback2)

        #-----------
        # Publishers
        #-----------
        self.pub_vel_cmd = rospy.Publisher('cmdvel', Twist, queue_size = 1)

    def PoseCallback1(self, msg):
        self.current_pose1.pose.pose = msg.pose.pose
        position = [self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z]




