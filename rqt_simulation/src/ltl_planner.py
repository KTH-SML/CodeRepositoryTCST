#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
import sys
import time
import numpy as np
from copy import deepcopy


from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32, PointStamped, PoseArray, Pose, Point

from std_msgs.msg import Bool, String

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from rqt_simulation_msgs.msg import Sense, TemporaryTask

from math import pi as PI
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, euler_from_matrix

from tf2_ros import tf2_ros, TransformListener
import tf2_geometry_msgs

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from std_srvs.srv import Empty

from pyquaternion import Quaternion


from ltl_tools.FTSLoader import FTSLoader
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner
from ltl_tools.automaton_vis import plot_automaton
from ltl_tools.temporary_task import temporaryTask
from ltl_tools.discrete_plan import initial_state_given_history, dijkstra_targets, dijkstra_plan_optimal
from ltl_tools.product import ProdAut_Run
import visualize_fts

class LtlPlannerNode(object):
    def __init__(self):
        self.node_name = "LTL Planner"
        self.active = False
        self.beta = 10
        self.robot_pose = PoseWithCovarianceStamped()
        self.hard_task = ''
        self.soft_task = ''
        self.temporary_task_ = temporaryTask()
        self.robot_name = rospy.get_param('robot_name')
        self.agent_type = rospy.get_param('agent_type')
        scenario_file = rospy.get_param('scenario_file')
        self.replan_timer = rospy.Time.now()

        self.last_current_pose = PoseStamped()
        self.clear_costmap = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

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
        # Goal pose for aerial vehicle
        if self.agent_type == 'aerial':
            self.GoalPublisher = rospy.Publisher('command/pose', PoseStamped, queue_size = 1)

        #------------
        # Subscribers
        #------------
        if self.agent_type == 'ground':
            #localization_topic = 'amcl_pose'
            localization_topic = 'pose_gui'
        elif self.agent_type == 'aerial':
            localization_topic = 'ground_truth/pose_with_covariance'
        self.sub_amcl_pose = rospy.Subscriber(localization_topic, PoseWithCovarianceStamped, self.PoseCallback)
        # trigger start from GUI
        self.sub_active_flag = rospy.Subscriber('/planner_active', Bool, self.SetActiveCallback)
        # initial position from GUI
        self.sub_init_pose = rospy.Subscriber('init_pose', Pose, self.GetInitPoseCallback)
        # task from GUI
        self.sub_soft_task = rospy.Subscriber('soft_task', String, self.SoftTaskCallback)
        self.sub_hard_task = rospy.Subscriber('hard_task', String, self.HardTaskCallback)
        self.sub_temp_task = rospy.Subscriber('temporary_task', TemporaryTask, self.TemporaryTaskCallback)
        # environment sense
        self.sub_sense = rospy.Subscriber('/environment', Sense, self.SenseCallback)
        # clear costmap manually from GUI
        self.sub_clear = rospy.Subscriber('clear_costmap', Bool, self.ClearCallback)

        ####### Wait 3 seconds to receive the initial position from the GUI
        usleep = lambda x: time.sleep(x)
        usleep(3)
        self.robot_motion.set_initial(self.init_pose)
        rospy.loginfo('%s : %s : The inital hard task is: %s' % (self.node_name, self.robot_name, self.hard_task))
        rospy.loginfo('%s : %s : The inital soft task is: %s' % (self.node_name, self.robot_name, self.soft_task))

        ####### robot information
        self.full_model = MotActModel(self.robot_motion, self.robot_action)
        self.planner = ltl_planner(self.full_model, self.hard_task, self.soft_task)
        ####### initial plan synthesis
        self.planner.optimal(10, 'static')

        ### Publish plan for GUI
        prefix_msg = self.plan_msg_builder(self.planner.run.line, rospy.Time.now())
        self.PrefixPlanPublisher.publish(prefix_msg)
        sufix_msg = self.plan_msg_builder(self.planner.run.loop, rospy.Time.now())
        self.SufixPlanPublisher.publish(sufix_msg)

                ### start up move_base
        self.navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.navi_goal = GoalMsg = MoveBaseGoal()

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
            elif self.agent_type == 'aerial':
                self.GoalPublisher.publish(self.navi_goal)

            print('Goal %s sent to %s.' %(str(self.next_move), str(self.robot_name)))

    def PoseCallback(self, current_pose):
        # PoseWithCovarianceStamped data from amcl_pose
        if self.active:
            if (rospy.Time.now()-self.replan_timer).to_sec() > 10.0:
                if self.planner.num_changed_regs > 0:
                    self.planner.replan(self.temporary_task_)
                    self.planner.num_changed_regs = 0
                    ### Publish plan for GUI
                    prefix_msg = self.plan_msg_builder(self.planner.run.line, rospy.Time.now())
                    self.PrefixPlanPublisher.publish(prefix_msg)
                    sufix_msg = self.plan_msg_builder(self.planner.run.loop, rospy.Time.now())
                    self.SufixPlanPublisher.publish(sufix_msg)

                self.replan_timer = rospy.Time.now()

            if self.agent_type == 'ground':
                self.checkMovement(current_pose)

            elif self.agent_type == 'aerial':
                position_error = sqrt((current_pose.pose.pose.position.x - self.navi_goal.pose.position.x)**2 + (current_pose.pose.pose.position.y - self.navi_goal.pose.position.y)**2 + (current_pose.pose.pose.position.z - self.navi_goal.pose.position.z)**2)
                current_R = quaternion_matrix([current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w])
                goal_R = quaternion_matrix([self.navi_goal.pose.orientation.x, self.navi_goal.pose.orientation.y, self.navi_goal.pose.orientation.z, self.navi_goal.pose.orientation.w])
                error_R = np.dot(current_R, goal_R.transpose())
                orientation_error = np.linalg.norm(euler_from_matrix(error_R))

            if self.agent_type == 'ground':
                if (self.navigation.get_state() == GoalStatus.SUCCEEDED):
                    print('Goal %s reached by %s.' %(str(self.next_move),str(self.robot_name)))
                    self.planner.find_next_move()
                    t = rospy.Time.now()-self.t0
                    print '----------Time: %.2f----------' %t.to_sec()
                    self.next_move = self.planner.next_move
                    print 'Robot %s next move is motion to %s' %(str(self.robot_name), str(self.next_move))
                    if (self.planner.index in self.temporary_task_.propos_planner_index) and (self.planner.segment == 'line'):
                        self.temporary_task_.remove_propos()
                    if (self.planner.index in self.temporary_task_.task_end_planner_index) and (self.planner.segment == 'line'):
                        self.temporary_task_.remove_task(self.planner.index)
                    self.navi_goal = self.FormatGoal(self.next_move, self.planner.index, t)
                    self.navigation.send_goal(self.navi_goal)
                    print('Goal %s sent to %s.' %(str(self.next_move), str(self.robot_name)))
                    rospy.loginfo('%s : %s : The agent is at state number %d and the segment is %s.' % (self.node_name, self.robot_name, self.planner.index, self.planner.segment))

            elif self.agent_type == 'aerial':
                if ((position_error < 0.15) and (orientation_error < 0.3)):
                    print('Goal %s reached by %s.' %(str(self.next_move),str(self.robot_name)))
                    self.planner.find_next_move()
                    t = rospy.Time.now()-self.t0
                    print '----------Time: %.2f----------' %t.to_sec()
                    self.next_move = self.planner.next_move
                    print 'Robot %s next move is motion to %s' %(str(self.robot_name), str(self.next_move))
                    self.navi_goal = self.FormatGoal(self.next_move, self.planner.index, t)
                    self.GoalPublisher.publish(self.navi_goal)
                    print('Goal %s sent to %s.' %(str(self.next_move), str(self.robot_name)))

    def GetInitPoseCallback(self, pose):
        if self.agent_type == 'aerial':
            pose.position.z = 2.0
        self.init_pose = ((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

    def SoftTaskCallback(self, soft_task):
        self.soft_task = soft_task.data

    def HardTaskCallback(self, hard_task):
        self.hard_task = hard_task.data

    def TemporaryTaskCallback(self, temporary_task):
        current_state = list(initial_state_given_history(self.planner.product, self.planner.run_history, self.planner.run, self.planner.index))
        current_state = current_state[0]
        self.temporary_task_.add_task(temporary_task)
        self.temporary_task_.make_combination_set()
        run_temp = self.temporary_task_.find_temporary_run(current_state, self.planner.product)

        end_temporary = set()
        end_temporary.add(run_temp.prefix[-1])

        new_run, time = dijkstra_plan_optimal(self.planner.product, self.beta, end_temporary)
        self.planner.index = 0
        self.planner.segment = 'line'
        self.planner.trace = []
        self.planner.run_history = []

        prefix = run_temp.prefix[0:-1]+new_run.prefix
        precost = run_temp.precost+new_run.precost
        suffix = run_temp.suffix+new_run.suffix
        sufcost = new_run.sufcost
        totalcost = precost + self.beta*sufcost
        self.planner.run = ProdAut_Run(self.planner.product, prefix, precost, suffix, sufcost, totalcost)

        ### Publish plan for GUI
        prefix_msg = self.plan_msg_builder(self.planner.run.line, rospy.Time.now())
        self.PrefixPlanPublisher.publish(prefix_msg)
        sufix_msg = self.plan_msg_builder(self.planner.run.loop, rospy.Time.now())
        self.SufixPlanPublisher.publish(sufix_msg)



    def SenseCallback(self, sense):
        regions = {}
        for i in range(0, len(sense.rois)):
            pose_tuple = self.pose_to_tuple(sense.rois[i].pose)
            label = []
            for j in range(0, len(sense.rois[i].propos_satisfied)):
                label.append(sense.rois[i].propos_satisfied[j].data)
            print(label)
            regions.update({pose_tuple : set(label)})
        sense_info = {'regions' : regions}
        #print sense.edges
        add_edges = []
        del_edges = []
        for i in range(0, len(sense.edges)):
            if sense.edges[i].add.data:
                add_edges.append((self.pose_to_tuple(sense.edges[i].start_pose), self.pose_to_tuple(sense.edges[i].target_pose)))
            else:
                del_edges.append((self.pose_to_tuple(sense.edges[i].start_pose), self.pose_to_tuple(sense.edges[i].target_pose)))
        sense_info.update({'edge' : [add_edges, del_edges]})
        self.planner.update_knowledge(sense_info)
        if self.planner.num_changed_regs > 10:
            self.planner.replan(self.temporary_task_)
            self.planner.num_changed_regs = 0
        elif len(del_edges) > 0:
            self.planner.validate_and_revise()

        ### Publish plan for GUI
        prefix_msg = self.plan_msg_builder(self.planner.run.line, rospy.Time.now())
        self.PrefixPlanPublisher.publish(prefix_msg)
        sufix_msg = self.plan_msg_builder(self.planner.run.loop, rospy.Time.now())
        self.SufixPlanPublisher.publish(sufix_msg)


    def ClearCallback(self, msg):
        if msg.data:
            rospy.loginfo('%s : %s : Costmap has been cleared and the current goal is resend.' % (self.node_name, self.robot_name))
            self.clear_costmap()
            usleep = lambda x: time.sleep(x)
            usleep(1)
            self.navigation.send_goal(self.navi_goal)

    def checkMovement(self, msg):
        msg_pose_rounded = Pose()
        msg_pose_rounded.position.x = round(msg.pose.pose.position.x - 0.005, 2)
        msg_pose_rounded.position.y = round(msg.pose.pose.position.y - 0.005, 2)
        msg_pose_rounded.position.z = round(msg.pose.pose.position.z - 0.005, 2)

        msg_pose_rounded.orientation.w = round(msg.pose.pose.orientation.w - 0.005, 2)
        msg_pose_rounded.orientation.x = round(msg.pose.pose.orientation.x - 0.005, 2)
        msg_pose_rounded.orientation.y = round(msg.pose.pose.orientation.y - 0.005, 2)
        msg_pose_rounded.orientation.z = round(msg.pose.pose.orientation.z - 0.005, 2)

        moved_distance = sqrt((msg.pose.pose.position.x - self.last_current_pose.pose.position.x)**2 + (msg.pose.pose.position.y - self.last_current_pose.pose.position.y)**2 +(msg.pose.pose.position.z - self.last_current_pose.pose.position.z)**2)
        current_R = quaternion_matrix([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        last_R = quaternion_matrix([self.last_current_pose.pose.orientation.x, self.last_current_pose.pose.orientation.y, self.last_current_pose.pose.orientation.z, self.last_current_pose.pose.orientation.w])
        performed_rotation_R = np.dot(current_R.transpose(), last_R)
        performed_rotation = np.linalg.norm(euler_from_matrix(performed_rotation_R))

        if (moved_distance > 0.05) or (performed_rotation > 0.1):
            self.last_current_pose.header.stamp = rospy.Time.now()

        if (rospy.Time.now() - self.last_current_pose.header.stamp).to_sec() > 5.0:
            rospy.loginfo('%s : %s : Costmap has been cleared and the current goal is resend.' % (self.node_name, self.robot_name))
            self.clear_costmap()
            usleep = lambda x: time.sleep(x)
            usleep(1)
            self.navigation.send_goal(self.navi_goal)
            self.last_current_pose.header.stamp = rospy.Time.now()
        self.last_current_pose.pose = deepcopy(msg_pose_rounded)

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
        elif self.agent_type == 'aerial':
            GoalMsg = PoseStamped()
            GoalMsg.header.seq = index
            GoalMsg.header.stamp = time_stamp
            GoalMsg.header.frame_id = 'map'
            GoalMsg.pose.position.x = goal[0][0]
            GoalMsg.pose.position.y = goal[0][1]
            GoalMsg.pose.position.z = 2.0

        return GoalMsg

    def plan_msg_builder(self, plan, time_stamp):
        plan_msg = PoseArray()
        plan_msg.header.stamp = time_stamp
        for n in plan:
            pose = Pose()
            pose.position.x = n[0][0][0]
            pose.position.y = n[0][0][1]
            pose.position.z = n[0][0][2]
            plan_msg.poses.append(pose)
        return plan_msg

    def pose_to_tuple(self, pose):
        tuple = ((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
        return tuple

    def euclidean_distance(self, position1, position2):
        return (sqrt((position1[0]-position2[0])**2+(position1[1]-position2[1])**2+(position1[2]-position2[2])**2))

if __name__ == '__main__':
    rospy.init_node('ltl_planner',anonymous=False)
    ltl_planner_node = LtlPlannerNode()
    rospy.spin()
