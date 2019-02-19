#!/usr/bin/env python
import roslib, rospy
roslib.load_manifest('ms1_kth')
import sys

import time

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from math import pi as PI
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler



def PoseCallback(posedata):
    # PoseWithCovarianceStamped data from amcl_pose
    global robot_pose # [time, [x,y,yaw]]
    header = posedata.header
    pose = posedata.pose
    if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
        # more recent pose data received
        robot_pose[0] = header.stamp
        # TODO: maybe add covariance check here?
        # print('robot position update!')
        euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
        robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
    return robot_pose



#========================================
global robot_pose
rospy.init_node('amcl_pose_to_Euler')


INITIAL = [10.0, 5.0, PI*0]

robot_pose = [None, INITIAL]
#----------
#publish to
#----------

#----------
#subscribe to
#----------
rospy.Subscriber('robot_0/amcl_pose', PoseWithCovarianceStamped, PoseCallback)

#----------
#main loop
#----------
t0 = rospy.Time.now()
while not rospy.is_shutdown():
    try:
        t = rospy.Time.now()-t0
        print '----------Time: %.2f----------' %t.to_sec()
        rospy.sleep(1)
        print 'robot pose %s' %str(robot_pose)
    except rospy.ROSInterruptException:
        pass
      
