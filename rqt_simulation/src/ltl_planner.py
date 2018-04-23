#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
import sys
import time
import numpy as np


from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32, PointStamped, PoseArray, Pose, Point

from std_msgs.msg import Bool, String

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from ms1_msgs.msg import Humans, ActionSeq

from math import pi as PI
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, euler_from_matrix
#from tf2_ros import tf2_ros, TransformListener

from tf2_ros import tf2_ros, TransformListener
import tf2_geometry_msgs

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from pyquaternion import Quaternion


from ltl_tools.FTSLoader import FTSLoader
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner
import visualize_fts

class LtlPlannerNode(object):
    def __init__(self):
        #[self.robot_motion, self.init_pose, self.robot_action, self.robot_task] = robot_model
        self.node_name = "LTL Planner"
        self.active = False
        self.robot_pose = PoseWithCovarianceStamped()
        self.hard_task = ''
        self.soft_task = ''
        self.robot_name = rospy.get_param('robot_name')
        self.agent_type = rospy.get_param('agent_type')
        scenario_file = rospy.get_param('scenario_file')

        robot_model = FTSLoader(scenario_file)
        [self.robot_motion, self.init_pose, self.robot_action, self.robot_task] = robot_model.robot_model

        #-----------
        # Publishers
        #-----------
        self.InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
        # Synthesised prefix plan Publisher
        self.PrefixPlanPublisher = rospy.Publisher('prefix_plan', PoseArray, queue_size = 1)
        # Synthesised sufix plan Publisher
        self.SufixPlanPublisher = rospy.Publisher('sufix_plan', PoseArray, queue_size = 1)
        # Goal pose for arial vehicle
        if self.agent_type == 'arial':
            self.GoalPublisher = rospy.Publisher('command/pose', PoseStamped, queue_size = 1)

        #------------
        # Subscribers
        #------------
        if self.agent_type == 'ground':
            #localization_topic = 'amcl_pose'
            localization_topic = 'pose_gui'
        elif self.agent_type == 'arial':
            localization_topic = 'ground_truth/pose_with_covariance'
        self.sub_amcl_pose = rospy.Subscriber(localization_topic, PoseWithCovarianceStamped, self.PoseCallback)
        # trigger start from GUI
        self.sub_active_flag = rospy.Subscriber('/planner_active', Bool, self.SetActiveCallback)
        # initial position from GUI
        self.sub_init_pose = rospy.Subscriber('init_pose', Pose, self.GetInitPoseCallback)
        # task from GUI
        self.sub_soft_task = rospy.Subscriber('soft_task', String, self.SoftTaskCallback)
        self.sub_hard_task = rospy.Subscriber('hard_task', String, self.HardTaskCallback)

        ####### Wait 3 seconds to receive the initial position from the GUI
        usleep = lambda x: time.sleep(x)
        usleep(3)
        self.robot_motion.set_initial(self.init_pose)
        print(self.hard_task)
        print(self.soft_task)

        ####### robot information
        self.full_model = MotActModel(self.robot_motion, self.robot_action)
        self.planner = ltl_planner(self.full_model, self.hard_task, self.soft_task)
        ####### initial plan synthesis
        self.planner.optimal(10)
        ### Publish plan for GUI
        prefix_msg = self.plan_msg_builder(self.planner.run.line, rospy.Time.now())
        self.PrefixPlanPublisher.publish(prefix_msg)
        sufix_msg = self.plan_msg_builder(self.planner.run.loop, rospy.Time.now())
        self.SufixPlanPublisher.publish(sufix_msg)
        ### start up move_base
        self.navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.navi_goal = GoalMsg = MoveBaseGoal()
        #rospy.loginfo("wait for the move_base action server to come up")
        #allow up to 5 seconds for the action server to come up
        #navigation.wait_for_server(rospy.Duration(5))
        #wait for the action server to come up
        #self.navigation.wait_for_server()

    def SetActiveCallback(self, state):
        self.active = state.data
        if self.active:
            self.t0 = rospy.Time.now()
            t = rospy.Time.now()-self.t0
            print '----------Time: %.2f----------' %t.to_sec()
            self.next_move = self.planner.next_move
            print 'Robot %s next move is motion to %s' %(str(self.robot_name), str(self.next_move))
            self.navi_goal = self.FormatGoal(self.next_move, self.planner.index, t)

            if self.agent_type == 'ground':
                self.navigation.send_goal(self.navi_goal)
            elif self.agent_type == 'arial':
                self.GoalPublisher.publish(self.navi_goal)

            print('Goal %s sent to %s.' %(str(self.next_move), str(self.robot_name)))

    def PoseCallback(self, current_pose):
        # PoseWithCovarianceStamped data from amcl_pose
        if self.active:
            if self.agent_type == 'ground':
                position_error = sqrt((current_pose.pose.pose.position.x - self.navi_goal.target_pose.pose.position.x)**2 + (current_pose.pose.pose.position.y - self.navi_goal.target_pose.pose.position.y)**2 + (current_pose.pose.pose.position.z - self.navi_goal.target_pose.pose.position.z)**2)
                current_R = quaternion_matrix([current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w])
                goal_R = quaternion_matrix([self.navi_goal.target_pose.pose.orientation.x, self.navi_goal.target_pose.pose.orientation.y, self.navi_goal.target_pose.pose.orientation.z, self.navi_goal.target_pose.pose.orientation.w])

                #current_euler = euler_from_quaternion([current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w])
                #goal_euler = euler_from_quaternion([self.navi_goal.target_pose.pose.orientation.x, self.navi_goal.target_pose.pose.orientation.y, self.navi_goal.target_pose.pose.orientation.z, self.navi_goal.target_pose.pose.orientation.w])
            elif self.agent_type == 'arial':
                position_error = sqrt((current_pose.pose.pose.position.x - self.navi_goal.pose.position.x)**2 + (current_pose.pose.pose.position.y - self.navi_goal.pose.position.y)**2 + (current_pose.pose.pose.position.z - self.navi_goal.pose.position.z)**2)
                current_R = quaternion_matrix([current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w])
                goal_R = quaternion_matrix([self.navi_goal.pose.orientation.x, self.navi_goal.pose.orientation.y, self.navi_goal.pose.orientation.z, self.navi_goal.pose.orientation.w])

                #current_euler = euler_from_quaternion([current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w])
                #goal_euler = euler_from_quaternion([self.navi_goal.pose.orientation.x, self.navi_goal.pose.orientation.y, self.navi_goal.pose.orientation.z, self.navi_goal.pose.orientation.w])

            error_R = np.dot(current_R, goal_R.transpose())
            orientation_error = np.linalg.norm(euler_from_matrix(error_R))
            #orientation_error = current_euler[0] - goal_euler[0] + current_euler[1] - goal_euler[1]  + current_euler[2] - goal_euler[2]
            #print('pos_err')
            #print(position_error)
            #print('orien_err')
            #print(orientation_error)
            if self.agent_type == 'ground':
                #print('planner')
                if ((position_error < 0.2) and (orientation_error < 0.8)) or (self.navigation.get_state() == GoalStatus.SUCCEEDED):
                #if ((position_error < 0.2)) or (self.navigation.get_state() == GoalStatus.SUCCEEDED):
                    print('Goal %s reached by %s.' %(str(self.next_move),str(self.robot_name)))
                    self.planner.find_next_move()
                    t = rospy.Time.now()-self.t0
                    print '----------Time: %.2f----------' %t.to_sec()
                    self.next_move = self.planner.next_move
                    print 'Robot %s next move is motion to %s' %(str(self.robot_name), str(self.next_move))
                    self.navi_goal = self.FormatGoal(self.next_move, self.planner.index, t)
                    if self.agent_type == 'ground':
                        self.navigation.send_goal(self.navi_goal)
                    elif self.agent_type == 'arial':
                        self.GoalPublisher.publish(self.navi_goal)
                    print('Goal %s sent to %s.' %(str(self.next_move), str(self.robot_name)))
            elif self.agent_type == 'arial':
                if ((position_error < 0.15) and (orientation_error < 0.3)):
                    print('Goal %s reached by %s.' %(str(self.next_move),str(self.robot_name)))
                    self.planner.find_next_move()
                    t = rospy.Time.now()-self.t0
                    print '----------Time: %.2f----------' %t.to_sec()
                    self.next_move = self.planner.next_move
                    print 'Robot %s next move is motion to %s' %(str(self.robot_name), str(self.next_move))
                    self.navi_goal = self.FormatGoal(self.next_move, self.planner.index, t)
                    if self.agent_type == 'ground':
                        self.navigation.send_goal(self.navi_goal)
                    elif self.agent_type == 'arial':
                        self.GoalPublisher.publish(self.navi_goal)
                    print('Goal %s sent to %s.' %(str(self.next_move), str(self.robot_name)))

    def GetInitPoseCallback(self, pose):
        if self.agent_type == 'arial':
            pose.position.z = 2.0
        self.init_pose = ((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

    def SoftTaskCallback(self, soft_task):
        self.soft_task = soft_task.data

    def HardTaskCallback(self,hard_task):
        self.hard_task = hard_task.data

    def FormatGoal(self, goal, index, time_stamp):
        if self.agent_type == 'ground':
            GoalMsg = MoveBaseGoal()
            GoalMsg.target_pose.header.seq = index
            GoalMsg.target_pose.header.stamp = time_stamp
            GoalMsg.target_pose.header.frame_id = 'map'
            GoalMsg.target_pose.pose.position.x = goal[0][0]
            GoalMsg.target_pose.pose.position.y = goal[0][1]
            #quaternion = quaternion_from_euler(0, 0, goal[2])
            GoalMsg.target_pose.pose.orientation.x = goal[1][1]
            GoalMsg.target_pose.pose.orientation.y = goal[1][2]
            GoalMsg.target_pose.pose.orientation.z = goal[1][3]
            GoalMsg.target_pose.pose.orientation.w = goal[1][0]
        elif self.agent_type == 'arial':
            GoalMsg = PoseStamped()
            GoalMsg.header.seq = index
            GoalMsg.header.stamp = time_stamp
            GoalMsg.header.frame_id = 'map'
            GoalMsg.pose.position.x = goal[0][0]
            GoalMsg.pose.position.y = goal[0][1]
            GoalMsg.pose.position.z = 2.0

            #quaternion = quaternion_from_euler(0, 0, goal[2])
            #GoalMsg.pose.orientation.x = goal[1][1]
            #GoalMsg.pose.orientation.y = goal[1][2]
            #GoalMsg.pose.orientation.z = goal[1][3]
            #GoalMsg.pose.orientation.w = goal[1][0]
        return GoalMsg

    def plan_msg_builder(self, plan, time_stamp):
        plan_msg = PoseArray()
        plan_msg.header.stamp = time_stamp
        print(plan)
        for n in plan:
            pose = Pose()
            pose.position.x = n[0][0][0]
            pose.position.y = n[0][0][1]
            pose.position.z = n[0][0][2]
            plan_msg.poses.append(pose)
        return plan_msg

    def convert_pose_from_map_to_mocap(self, pose, tf_mocap_to_map):
        M_trans_M_R = Quaternion([0, pose.position.x, pose.position.y, pose.position.z])
        M_quat_M_R = Quaternion([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])

        Q_trans_Q_M = Quaternion([0, tf_mocap_to_map.position.x, tf_mocap_to_map.position.y, tf_mocap_to_map.position.z])
        Q_quat_Q_M = Quaternion([tf_mocap_to_map.orientation.w, tf_mocap_to_map.orientation.x, tf_mocap_to_map.orientation.y, tf_mocap_to_map.orientation.z])

        Q_trans_Q_R = Q_quat_Q_M * M_trans_M_R * Q_quat_Q_M.inversion
        Q_quat_Q_R = Q_quat_Q_M * M_quat_M_R

        Q_pose_Q_R = Pose()
        Q_pose_Q_R.position.x = Q_trans_Q_R[1]
        Q_pose_Q_R.position.y = Q_trans_Q_R[2]
        Q_pose_Q_R.position.z = Q_trans_Q_R[3]
        Q_pose_Q_R.orientation.w = Q_quat_Q_R[0]
        Q_pose_Q_R.orientation.x = Q_quat_Q_R[1]
        Q_pose_Q_R.orientation.y = Q_quat_Q_R[2]
        Q_pose_Q_R.orientation.z = Q_quat_Q_R[3]

        return Q_pose_Q_R




if __name__ == '__main__':
    rospy.init_node('ltl_planner',anonymous=False)
    ltl_planner_node = LtlPlannerNode()
    rospy.spin()
