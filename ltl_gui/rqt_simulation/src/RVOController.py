#!/usr/bin/env python

import roslib
import numpy
import Queue
import rospy
import sys
from math import cos, sin, atan2, sqrt

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry

from pyquaternion import Quaternion

from RVO_Py_MAS.RVO import RVO_update


class RVOControllerNode(object):
    def __init__(self):
        self.node_name = "RVO Controller"
        self.current_pose1 = PoseWithCovarianceStamped()
        self.current_pose2 = PoseWithCovarianceStamped()
        self.current_twist1 = Twist()
        self.current_twist2 = Twist()
        self.current_position_list = [[self.current_pose1.pose.pose.position.x, self.current_pose1.pose.pose.position.y], [self.current_pose2.pose.pose.position.x, self.current_pose2.pose.pose.position.y]]
        self.current_vel_des_list = [[self.current_twist1.linear.x, self.current_twist1.linear.y], [self.current_twist2.linear.x, self.current_twist2.linear.y]]
        self.current_vel_list = [[self.current_twist1.linear.x, self.current_twist1.linear.y], [self.current_twist2.linear.x, self.current_twist2.linear.y]]

        self.tau = 1.0/20.0 * 3.0
        self.L = 0.3

        #------------------------------
        #define workspace model
        self.ws_model = dict()
        #robot radius
        self.ws_model['robot_radius'] = 0.2
        #circular obstacles, format [x,y,rad]
        # no obstacles
        self.ws_model['circular_obstacles'] = []
        # with obstacles
        #ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
        #rectangular boundary, format [x,y,width/2,heigth/2]
        self.ws_model['boundary'] = []

        #------------
        # Subscribers
        #------------
        # Get current pose
        #self.sub_pose = rospy.Subscriber('ground_truth/pose_with_covariance', PoseWithCovarianceStamped, self.PoseCallback)
        self.sub_odom1 = rospy.Subscriber('/robot1/odom', Odometry, self.OdomCallback1)
        #self.sub_pose2 = rospy.Subscriber('/robot2/odom', Odometry, self.OdomCallback2)
        self.sub_pose1 = rospy.Subscriber('/robot1/amcl_pose', PoseWithCovarianceStamped, self.PoseCallback1)
        # Get desired velocity
        self.sub_vel_des1 = rospy.Subscriber('/robot1/mobile_base/commands/velocity_raw', Twist, self.vel_des_callback1)
        #self.sub_vel2 = rospy.Subscriber('/robot2/mobile_base/commands/velocity', Twist, self.vel_callback2)

        #-----------
        # Publishers
        #-----------
        self.pub_vel_cmd1 = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size = 1)

    def PoseCallback1(self, msg):
        self.current_pose1.pose.pose = msg.pose.pose
        #self.current_twist1 = msg.twist.twist
        position = [self.current_pose1.pose.pose.position.x, self.current_pose1.pose.pose.position.y]
        self.current_position_list[0] = position

        # Get orientation from pose
        quaternion = Quaternion(self.current_pose1.pose.pose.orientation.w, self.current_pose1.pose.pose.orientation.x, self.current_pose1.pose.pose.orientation.y, self.current_pose1.pose.pose.orientation.z)
        rot_axis = quaternion.axis
        theta = quaternion.angle * rot_axis[2]

        current_vel = [self.current_twist1.linear.x * cos(theta), self.current_twist1.linear.x * sin(theta)]
        self.current_vel_list[0] = current_vel


    def PoseCallback2(self, msg):
        self.current_pose2.pose.pose = msg.pose.pose
        position = [self.current_pose2.pose.pose.position.x, self.current_pose2.pose.pose.position.y]
        self.current_position_list[1] = position

    def OdomCallback1(self, msg):
        self.current_twist1 = msg.twist.twist

    def vel_des_callback1(self, msg):

        # Get orientation from pose
        quaternion = Quaternion(self.current_pose1.pose.pose.orientation.w, self.current_pose1.pose.pose.orientation.x, self.current_pose1.pose.pose.orientation.y, self.current_pose1.pose.pose.orientation.z)
        rot_axis = quaternion.axis
        theta = quaternion.angle * rot_axis[2]
        print('---------------------')
        print(theta)

        twist_world = Twist()
        twist_world.linear.x = msg.linear.x * cos(theta)
        twist_world.linear.y = msg.linear.x * sin(theta)

        vel_des = [msg.linear.x * cos(theta), msg.linear.x * sin(theta)]
        print(vel_des)

        self.current_vel_des_list[0] = vel_des

        updated_vel = RVO_update(self.current_position_list, self.current_vel_des_list, self.current_vel_list, self.ws_model)
        print(updated_vel)

        if sqrt(updated_vel[0][0]**2 + updated_vel[0][1]**2) == 0:
            theta_des = theta
        else:
            theta_des = (atan2(updated_vel[0][1], updated_vel[0][0]))
        print(theta_des)
        delta_theta = theta_des - theta

        print(delta_theta)

        updated_twist = Twist()
        updated_twist.linear.x = updated_vel[0][0] / (cos(theta_des))
        updated_twist.angular.z = delta_theta/self.tau# + 2.0/self.L * (self.current_vel_list[0][1]/sin(theta) - self.current_vel_list[0][0]/cos(theta))

        self.pub_vel_cmd1.publish(updated_twist)




if __name__ == '__main__':
    rospy.init_node('RVO_controller', anonymous=False)
    RVO_controller_node = RVOControllerNode()
    rospy.spin()





