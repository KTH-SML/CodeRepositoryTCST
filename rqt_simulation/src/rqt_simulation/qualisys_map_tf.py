#!/usr/bin/env python
import roslib
import os
import yaml
import codecs
import rospkg
import numpy
import Queue
import rospy
import sys
import time
import tf
import roslaunch
from math import sqrt

from geometry_msgs.msg import PoseStamped, Transform, Pose
from nav_msgs.msg import Odometry

from pyquaternion import Quaternion


class QualisysMapTfNode(object):
    def __init__(self):
        self.node_name = "Qualisys to Map TF"

        model_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'qualisys_models.yaml')
        stream = file(model_file, 'r')
        data = yaml.load(stream)
        models = data['models']

        self.start_stamp = rospy.Time.now().to_sec()

        self.tf_qualisys_map = Transform()
        self.tf_qualisys_map.translation.x = rospy.get_param('~trans_x')
        self.tf_qualisys_map.translation.y = rospy.get_param('~trans_y')
        self.tf_qualisys_map.translation.z = rospy.get_param('~trans_z')
        self.tf_qualisys_map.rotation.w = rospy.get_param('~orient_w')
        self.tf_qualisys_map.rotation.x = rospy.get_param('~orient_x')
        self.tf_qualisys_map.rotation.y = rospy.get_param('~orient_y')
        self.tf_qualisys_map.rotation.z = rospy.get_param('~orient_z')

        self.publisher_pose_dict = {}

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_qualisys_odom_list = []

        self.first_pose_dict = {}
        self.odom = Pose()

        for i in range(0, len(models)):
            self.first_pose_dict.update({models[i] : True})
            roslaunch_qualisys_odom_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'launch', 'qualisys_odom.launch')]))
            sys.argv.append('model:=' + models[i])
            self.sub_qualysis_pose = rospy.Subscriber(models[i] + '/qualisys_pose', PoseStamped, self.pose_cb, models[i])
            self.publisher_pose_dict.update({models[i] : rospy.Publisher(models[i] + '/qualisys_pose_map', PoseStamped, queue_size = 1)})
            roslaunch_qualisys_odom_list[i].start()
            del sys.argv[2:len(sys.argv)]

    def pose_cb(self, msg, source):
        tf_br = tf.TransformBroadcaster()
        M_pose_M_R = PoseStamped()
        M_pose_M_R.header = msg.header
        M_pose_M_R.header.frame_id = 'map'
        M_pose_M_R.pose = self.convert_pose_from_frame1_to_frame2(msg.pose, self.tf_qualisys_map)
        M_pose_M_R.pose.position.z = 0.0
        self.publisher_pose_dict[source].publish(M_pose_M_R)

        if self.first_pose_dict[source] and (msg.header.stamp.to_sec() > self.start_stamp):
            self.odom = M_pose_M_R.pose
            self.tf_map_odom = Transform()
            self.tf_map_odom.translation = self.odom.position
            self.tf_map_odom.rotation = self.odom.orientation
            self.first_pose_dict[source] = False

        if self.first_pose_dict[source] == False:
            O_pose_O_R = PoseStamped()
            O_pose_O_R.header = msg.header
            O_pose_O_R.header.frame_id = 'map'
            O_pose_O_R.pose = self.convert_pose_from_frame1_to_frame2(M_pose_M_R.pose, self.tf_map_odom)
            tf_br.sendTransform((O_pose_O_R.pose.position.x, O_pose_O_R.pose.position.y, O_pose_O_R.pose.position.z), (O_pose_O_R.pose.orientation.x, O_pose_O_R.pose.orientation.y, O_pose_O_R.pose.orientation.z, O_pose_O_R.pose.orientation.w), rospy.Time.now(), source + '/base_footprint', source + '/qualisys_odom')
            tf_br.sendTransform((self.odom.position.x, self.odom.position.y, self.odom.position.z), (self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w), rospy.Time.now(), source + '/qualisys_odom', '/map')

    def convert_pose_from_frame1_to_frame2(self, pose, tf_frame1_to_frame2):
        trans_1_pose = Quaternion([0.0, pose.position.x, pose.position.y, pose.position.z])
        quat_1_pose = Quaternion([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        trans_1_2 = Quaternion([0.0, tf_frame1_to_frame2.translation.x, tf_frame1_to_frame2.translation.y, tf_frame1_to_frame2.translation.z])

        quat_1_2 = Quaternion([tf_frame1_to_frame2.rotation.w, tf_frame1_to_frame2.rotation.x, tf_frame1_to_frame2.rotation.y, tf_frame1_to_frame2.rotation.z])
        quat_2_1 = quat_1_2.inverse

        trans_2_pose = quat_2_1 * trans_1_pose * quat_2_1.inverse - quat_2_1 * trans_1_2 * quat_2_1.inverse
        quat_2_pose = quat_2_1 * quat_1_pose

        pose_2_pose = Pose()
        pose_2_pose.position.x = trans_2_pose[1]
        pose_2_pose.position.y = trans_2_pose[2]
        pose_2_pose.position.z = trans_2_pose[3]
        pose_2_pose.orientation.w = quat_2_pose[0]
        pose_2_pose.orientation.x = quat_2_pose[1]
        pose_2_pose.orientation.y = quat_2_pose[2]
        pose_2_pose.orientation.z = quat_2_pose[3]

        return pose_2_pose

    def invert_tf(self, tf):
        tf_trans = Quaternion([0, tf.translation.x, tf.translation.y, tf.translation.z])
        tf_quat = Quaternion([tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z])

        tf_quat_inv = tf_quat.inversion()
        tf_trans_inv = -(tf_quat_inv * tf_trans * tf_quat)

        tf_inv = Transform()
        tf_inv.translation.x = tf_trans_inv[1]
        tf_inv.translation.y = tf_trans_inv[2]
        tf_inv.translation.z = tf_trans_inv[3]
        tf_inv.rotation.w = tf_quat_inv[0]
        tf_inv.rotation.x = tf_quat_inv[1]
        tf_inv.rotation.y = tf_quat_inv[2]
        tf_inv.rotation.z = tf_quat_inv[3]

        return tf_inv


if __name__ == '__main__':
    rospy.init_node('qualisys_to_map_tf', anonymous=False)
    qualisys_to_map_tf_node = QualisysMapTfNode()
    rospy.spin()
