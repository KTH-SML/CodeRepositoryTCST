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

        #print(models)

        self.tf_qualisys_map = Transform()
        self.tf_qualisys_map.translation.x = 0.111
        self.tf_qualisys_map.translation.y = 0.105
        self.tf_qualisys_map.translation.z = 0.0
        self.tf_qualisys_map.rotation.w = 0.934
        self.tf_qualisys_map.rotation.x = 0.0
        self.tf_qualisys_map.rotation.y = 0.0
        self.tf_qualisys_map.rotation.z = 0.358

        self.publisher_pose_dict = {}
        #self.publisher_odom_dict = {}

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_qualisys_odom_list = []

        self.first_pose = True
        self.odom = Pose()

        #tf_br_list = []

        for i in range(0, len(models)):
            roslaunch_qualisys_odom_list.append(roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(rospkg.RosPack().get_path('qualisys'), 'launch', 'qualisys_odom.launch')]))
            sys.argv.append('model:=' + models[i])
            self.sub_qualysis_pose = rospy.Subscriber(models[i] + '/pose', PoseStamped, self.pose_cb, models[i])
            self.publisher_pose_dict.update({models[i] : rospy.Publisher(models[i] + '/map_pose', PoseStamped, queue_size = 1)})
            #self.publisher_pose_dict.update({models[i] : rospy.Publisher(models[i] + '/map_pose', PoseStamped, queue_size = 1)})
            roslaunch_qualisys_odom_list[i].start()
            del sys.argv[2:len(sys.argv)]
            #tf_br_list.append(tf.TransformBroadcaster())
            #tf_br_list[i].sendTransform((0.0, 0.0, -self.tf_qualisys_map.translation.z), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), models[i] + '/qualisys_odom', models[i])

        #tf_br = tf.TransformBroadcaster()
        #tf_br.sendTransform((0.0, 0.0, -self.tf_qualisys_map.translation.z), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), source + '/qualisys_odom', models[i])

    def pose_cb(self, msg, source):
        tf_br = tf.TransformBroadcaster()
        M_pose_M_R = PoseStamped()
        M_pose_M_R.header = msg.header
        M_pose_M_R.header.frame_id = 'map'
        M_pose_M_R.pose = self.convert_pose_from_frame1_to_frame2(msg.pose, self.tf_qualisys_map)
        M_pose_M_R.pose.position.z = 0.0
        #self.pub_map_pose.publish(M_pose_M_R)
        self.publisher_pose_dict[source].publish(M_pose_M_R)

        if self.first_pose:
            self.odom = M_pose_M_R.pose
            self.first_pose = False

        tf_br.sendTransform((M_pose_M_R.pose.position.x, M_pose_M_R.pose.position.y, M_pose_M_R.pose.position.z), (M_pose_M_R.pose.orientation.x, M_pose_M_R.pose.orientation.y, M_pose_M_R.pose.orientation.z, M_pose_M_R.pose.orientation.w), rospy.Time.now(), source + '/qualisys_footprint', 'map')
        #tf_br.sendTransform((self.tf_qualisys_map.translation.x, self.tf_qualisys_map.translation.y, self.tf_qualisys_map.translation.z, M_pose_M_R.pose.position.z), (self.tf_qualisys_map.rotation.x, self.tf_qualisys_map.rotation.y, self.tf_qualisys_map.rotation.z, self.tf_qualisys_map.rotation.w), rospy.Time.now(), source + '/qualisys_footprint', 'map')
        tf_br.sendTransform((self.odom.position.x, self.odom.position.y, self.odom.position.z), (self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w), rospy.Time.now(), source + '/qualisys_odom', 'map')
        #print('now')

        odom = Odometry()



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

