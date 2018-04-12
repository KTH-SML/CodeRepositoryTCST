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
from math import sqrt

from geometry_msgs.msg import PoseStamped, Transform, Pose

from pyquaternion import Quaternion


class QualisysMapTfNode(object):
    def __init__(self):
        self.node_name = "Qualisys to Map TF"

        model_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'qualisys_models.yaml')
        stream = file(model_file, 'r')
        data = yaml.load(stream)
        models = data['models']

        print(models)

        self.tf_qualisys_map = Transform()
        self.tf_qualisys_map.translation.x = 0.111
        self.tf_qualisys_map.translation.y = 0.105
        self.tf_qualisys_map.translation.z = 0.0
        self.tf_qualisys_map.rotation.w = 0.934
        self.tf_qualisys_map.rotation.x = 0.0
        self.tf_qualisys_map.rotation.y = 0.0
        self.tf_qualisys_map.rotation.z = 0.358

        self.publisher_dict = {}

        for i in range(0, len(models)):
            self.sub_qualysis_pose = rospy.Subscriber(models[i] + '/pose', PoseStamped, self.pose_cb, models[i])
            self.publisher_dict.update({models[i] : rospy.Publisher(models[i] + '/map_pose', PoseStamped, queue_size = 1)})

    def pose_cb(self, msg, source):
        tf_br = tf.TransformBroadcaster()
        M_pose_M_R = PoseStamped()
        M_pose_M_R.header = msg.header
        M_pose_M_R.pose = self.convert_pose_from_frame1_to_frame2(msg.pose, self.tf_qualisys_map)
        #self.pub_map_pose.publish(M_pose_M_R)
        self.publisher_dict[source].publish(M_pose_M_R)
        tf_br.sendTransform((M_pose_M_R.pose.position.x, M_pose_M_R.pose.position.y,M_pose_M_R.pose.position.z), (M_pose_M_R.pose.orientation.w, M_pose_M_R.pose.orientation.x, M_pose_M_R.pose.orientation.y, M_pose_M_R.pose.orientation.z), rospy.Time.now(), source + '/qualisys_odom', source)
        #print('now')


    def convert_pose_from_frame1_to_frame2(self, pose, tf_frame1_to_frame2):
        trans_1_pose = Quaternion([0.0, pose.position.x, pose.position.y, pose.position.z])
        quat_1_pose = Quaternion([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        #print(quat_1_pose)
        trans_1_2 = Quaternion([0.0, tf_frame1_to_frame2.translation.x, tf_frame1_to_frame2.translation.y, tf_frame1_to_frame2.translation.z])
        #print(trans_1_2)
        quat_1_2 = Quaternion([tf_frame1_to_frame2.rotation.w, tf_frame1_to_frame2.rotation.x, tf_frame1_to_frame2.rotation.y, tf_frame1_to_frame2.rotation.z])
        #print(quat_1_2)

        #tf_frame2_to_frame1 = self.invert_tf(tf_frame1_to_frame2)
        quat_2_1 = quat_1_2.inverse
        #print(quat_2_1)

        trans_2_pose = quat_2_1 * trans_1_pose * quat_2_1.inverse - quat_2_1 * trans_1_2 * quat_2_1.inverse
        #print(trans_2_pose)
        quat_2_pose = quat_2_1 * quat_1_pose

        #print(quat_2_pose)

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

