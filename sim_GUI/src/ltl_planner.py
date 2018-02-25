#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('sim_GUI')
import rospy
import sys
import time


from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32, PointStamped

from std_msgs.msg import Bool, String

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from ms1_msgs.msg import Humans, ActionSeq

from math import pi as PI
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from tf2_ros import tf2_ros, TransformListener

from tf2_ros import tf2_ros, TransformListener
import tf2_geometry_msgs

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus


from ltl_tools.fts_loader import robot_model, compute_poly
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner
import visualize_fts


def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


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
    return False

def DetectionCallback(detectdata):
    # human detection notification
    global human_detected
    print 'Manual detection data received: %s' %str(detectdata.data)
    if detectdata.data:
        print 'Human detected by manual data!'
        human_detected = True

def HumanCallback(humandata):
    # human detection from FORTH
    global human_data
    human_data = humandata
    print '---Human_data received---'

def ConfirmationCallback(confdata):
    global action_confirmation
    print 'confdata', confdata
    index = confdata.seq
    action = confdata.data
    print 'Action %s with index %s done!' %(action, index)
    action_confirmation = (int(index), str(action))

def GestureCallback(gesture_data):
    global gesture_detected
    print 'Gesture confdata', gesture_detected
    gesture_detected = gesture_data.data
    print 'Gesture %s detected!' %(gesture_detected)


def NaviCallback(navidata):
    global navi_result
    ID = navidata.status.goal_id.id
    status = navidata.status
    print 'Navigation %s status: %d'%(ID, status)
    navi_result = (ID, status)

def FormatGoal(goal, index, time_stamp):
    GoalMsg = MoveBaseGoal()
    GoalMsg.target_pose.header.seq = index
    GoalMsg.target_pose.header.stamp = time_stamp
    GoalMsg.target_pose.header.frame_id = 'map'
    GoalMsg.target_pose.pose.position.x = goal[0]
    GoalMsg.target_pose.pose.position.y = goal[1]
    quaternion = quaternion_from_euler(0, 0, goal[2])
    GoalMsg.target_pose.pose.orientation.x = quaternion[0]
    GoalMsg.target_pose.pose.orientation.y = quaternion[1]
    GoalMsg.target_pose.pose.orientation.z = quaternion[2]
    GoalMsg.target_pose.pose.orientation.w = quaternion[3]
    return GoalMsg

def SendAction(ActionPublisher, action, index):
    ActionMsg = ActionSeq()
    ActionMsg.seq = index
    ActionMsg.data = action
    ActionPublisher.publish(ActionMsg)

def SendPolygon(RoiPublisher, target_poly):
    PolyMsg = PolygonStamped()
    #PolyMsg.header.frame_id = "/xiton_rgb_optical_frame"
    PolyMsg.header.frame_id = "/map"
    poly_xyz = []
    Z = 0 # assume to be ground level
    if target_poly != 'None':
        for p_xy in target_poly:
            point = Point32()
            point.x, point.y, point.z = (p_xy[0],p_xy[1],Z)
            poly_xyz.append(point)
    else:
        point = Point32()
        point.x, point.y, point.z = (0,0,Z)
        poly_xyz.append(point)
    PolyMsg.polygon.points = poly_xyz
    RoiPublisher.publish(PolyMsg)

def transform(tf_listener, p):
    pt = PointStamped()
    tf_pt = PointStamped()
    pt.header.frame_id = '/map'
    pt.point.x, pt.point.y, pt.point.z = (p[0], p[1], 0)
    tf_pt = tf_listener.transformPoint('/xtion_rgb_optical_frame', pt)
    tf_p = [tf_pt.point.x, tf_pt.point.y]
    return tf_p

def determine_human_detected(human_data, tf_listener, HumanDebugPublisher, roi):
    if human_data.humans_num >= 1:
        print "Human detected, check if they are in ROI"
        for human in human_data.humans:
            pos = transform_pose(tf_listener, HumanDebugPublisher, human.pose, human_data.header.frame_id)
            if pos and roi:
                (min_x, min_y) = get_min_point(roi)
                (max_x, max_y) = get_max_point(roi)
                if (not min_x <= pos.x <= max_x and not min_y <= pos.y <= max_y):
                    print 'Human outside ROI'
                    continue # not even in bounding box
                if determine_inside((min_x, min_y), (pos.x,pos.y), roi):
                    print 'human within ROI based on human_data'
                    return True
                else:
                    print 'Human outside ROI'
    else:
        print 'No human based on human_data from sensors'
        return False

def transform_pose(tf_listener, HumanDebugPublisher, pose, src_frame):
        tf_time = rospy.Time()
        # if tf_listener.can_transform(src_frame, "map", tf_time):
        #         rospy.logwarn("transform from %s to %s is not available at time %s" % (src_frame, "map", str(tf_time)))
        #         return None
        try:
            transform = tf_listener.lookup_transform("map", src_frame, tf_time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("transform from %s to %s is not available at time %s" % (src_frame, "map", str(tf_time)))
            return None

        pose_src = PoseStamped(pose=pose)
        pose_src.header.frame_id = src_frame
        pose_src.header.stamp = tf_time
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_src, transform)
        SendHumanDebug(HumanDebugPublisher, pose_transformed)
        return pose_transformed.pose.position

def SendHumanDebug(HumanDebugPublisher, pose_transformed):
    HumanDebugPublisher.publish(pose_transformed)

def get_min_point(polygon):
        min_x = min(p[0] for p in polygon)
        min_y = min(p[1] for p in polygon)
        return (min_x - .1, min_y - .1)

def get_max_point(polygon):
        max_x = min(p[0] for p in polygon)
        max_y = max(p[1] for p in polygon)
        return (max_x + .1, max_y + .1)

def determine_inside(start, target, polygon):
        target_line = calc_cross(start, target)
        last_point = polygon[0]
        intersect_counter = 0
        for point in polygon[1:] + [last_point]:
                line = calc_cross(last_point, point)
                intersect_point = calc_cross(target_line, line)
                if (is_between(last_point, point, intersect_point)
                    and is_between(start, target, intersect_point)):
                        intersect_counter += 1
                last_point = point
        return bool(intersect_counter % 2)

def calc_cross(start, end):
        z = start[0] * end[1] - end[0] * start[1]
        return ((start[1] - end[1]) / z, (end[0] - start[0]) / z)

def is_between(start, end, target):
        (min_x, max_x) = sorted([start[0], end[0]])
        (min_y, max_y) = sorted([start[1], end[1]])
        return min_x <= target[0] <= max_x and min_y <= target[1] <= max_y

def determine_poly(goal_pose):
    width = 2 #m
    length = 2 #m
    angle = goal_pose[2]
    target_poly = compute_poly(goal_pose[0:2], angle, width, length)
    # loc_poly = []
    # for p in target_poly:
    #     tf_p = transform(p)
    #     loc_poly.append(tf_p)
    # print "Targeted ROI as polygon in camera frame sent as:", loc_poly
    # return loc_poly
    return target_poly


def planner(ts, init_pose, act, robot_task, robot_name='TIAGo'):
    global robot_pose
    global action_confirmation
    global navi_result
    global human_data
    global gesture_detected
    global human_detected
    robot_pose = [None, init_pose]
    human_detected = False
    gesture_detected = 'None'
    action_confirmation = (None, None)
    navi_result = (None, None)
    human_data = Humans()
    target_poly = None
    rospy.init_node('ltl_planner_%s' %robot_name)
    print 'Robot %s: ltl_planner started!' %(robot_name)
    visualize_fts.init()
    tf_listener = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_listener)
    ###### publish to
    #----------
    #publish to
    #----------
    InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
    # Interface to NTUA:
    ActionPublisher = rospy.Publisher('action/id', ActionSeq, queue_size = 100)
    # Interface to FORTH
    RoiPublisher = rospy.Publisher('perception_focus_area', PolygonStamped, queue_size = 100)
    HumanDebugPublisher = rospy.Publisher('human_pose_debug', PoseStamped, queue_size = 100)
    #----------
    #subscribe to
    #----------
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
    rospy.Subscriber('mission_update', String, GestureCallback)
    # interface 1 to FORTH
    # for testing
    rospy.Subscriber('human_detection', Bool, DetectionCallback)
    # for demo
    rospy.Subscriber('tracked_humans', Humans, HumanCallback)
    # interface 2 to NTUA
    rospy.Subscriber('action/result', ActionSeq, ConfirmationCallback)
    ####### robot information
    full_model = MotActModel(ts, act)
    planner = ltl_planner(full_model, robot_task[0], robot_task[1])
    ####### initial plan synthesis
    planner.optimal(10)
    ### start up move_base
    navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("wait for the move_base action server to come up")
    #allow up to 5 seconds for the action server to come up
    navigation.wait_for_server(rospy.Duration(5))
    roi = 'r2'
    action_index = 1
    grasp_done = False

    markers = []
    for n in full_model.graph["region"].nodes():
        prop = full_model.graph["region"].node[n]["label"]
        markers.extend(visualize_fts.create_marker(n, ','.join(prop)))
    visualize_fts.send_markers(markers)
    #######
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        try:
            t = rospy.Time.now()-t0
            print '----------Time: %.2f----------' %t.to_sec()
            next_move = planner.next_move
            if isinstance(next_move, str):
                if (str(next_move) == 'col_grasp'):
                    if  grasp_done == False:
                        next_move = 'grasp'
                    else:
                        next_move = 'col_grasp'
                print 'Robot %s next move is action [%s]' %(str(robot_name), str(next_move))
                SendAction(ActionPublisher, next_move, action_index)
                print 'Next action message sent for [%s] with index %d' %(str(next_move), action_index)
                # ------------------------------
                # Interface 2 to NTUA
                # rostopic pub /action/result roseus/StringStamped "header:
                #   seq: 1
                #   stamp:
                #     secs: 0
                #     nsecs: 0
                #   frame_id: ''
                # data: 'col_grasp'"
                print 'Waiting for confirmation of action [%s] completion, with index %d' %(next_move, action_index)
                while (not rospy.is_shutdown()) and ((action_confirmation[0] != action_index) or (action_confirmation[1] !=next_move)):
                    try:
                        rospy.sleep(1)
                        print 'received action_confirmation', action_confirmation
                        print 'Waiting for [action_index, next_move]', [action_index, next_move]
                        if (next_move == 'col_grasp') and (gesture_detected == 'stop'):
                            print 'Gesture %s received' %gesture_detected
                            break
                    except rospy.ROSInterruptException:
                        pass
                print 'Successful execution of action [%s] with index %d' %(next_move, action_index)
                action_index += 1
                if next_move == 'grasp':
                    next_move = 'col_grasp'
                    grasp_done = True
                elif (next_move == 'col_grasp') and (grasp_done == True):
                    planner.find_next_move()
                    grasp_done = False
            else:
                print 'Robot %s next move is motion to %s' %(str(robot_name), str(next_move))
                navi_goal = FormatGoal(next_move, planner.index, t)
                navigation.send_goal(navi_goal)
                print('Goal %s sent to %s.' %(str(next_move), str(robot_name)))
                ############
                #success = navigation.wait_for_result(rospy.Duration(60))
                while (not rospy.is_shutdown()) and (navigation.get_state() != GoalStatus.SUCCEEDED):
                    try:
                        ###############  check for model update
                        if roi in ts.node[next_move]['label']:
                            print '----------going to the ROI next----------'
                            if not target_poly:
                                target_poly = determine_poly(next_move)
                            print 'human_detected value', human_detected
                            if ((human_detected) or (determine_human_detected(human_data, tf_listener, HumanDebugPublisher, target_poly))):
                                print 'add hm to fts state label'
                                if 'hm' not in ts.node[next_move]['label']:
                                    planner.update_add('hm', roi)
                                    print 'Agent %s: human detection incorporated in map!' %robot_name
                                    planner.replan_simple(robot_pose[1])
                                    human_detected = False
                            # else:
                            #     # interface 2 to FORTH
                            #     # rostopic pub /human_detection std_msgs/Bool "data: false"
                            #     new_update = planner.update_remove('hm', roi)
                            #     if new_update:
                            #         print 'Agent %s: human non-detection incorporated in map!' %robot_name
                            #         planner.replan_simple(robot_pose[1])
                        ############
                        ############ send roi as focused area
                        if roi in ts.node[next_move]['label']:
                        #if True:
                            # if not tf_listener.can_transform("/map", "/xiton_rgb_optical_frame", rospy.Time(0)):
                            #     rospy.logwarn("transform from %s to %s is not available" % ("/map", "/xiton_rgb_optical_frame"))
                            #     return None
                            # transform = self._tf_listener.lookup_transform("/map", "/xiton_rgb_optical_frame", rospy.Time(0))
                            target_poly = determine_poly(next_move)
                            SendPolygon(RoiPublisher, target_poly)
                        else:
                            SendPolygon(RoiPublisher, 'None')
                        rospy.sleep(2)
                    except rospy.ROSInterruptException:
                        pass
                # ------------------------------
                print('Goal %s reached by %s.' %(str(next_move),str(robot_name)))
                planner.find_next_move()
        except rospy.ROSInterruptException:
            pass

'''
Useful commends here for testing:

rostopic pub /robot_0/human_detection std_msgs/Bool "{data: true}"

rostopic pub /robot_0/action/result ms1_msgs/ActionSeq "{seq: 1, data: "grasp"}"

rostopic pub /robot_0/action/result ms1_msgs/ActionSeq "{seq: 2, data: "col_grasp"}"

rostopic pub /robot_0/gesture_detected std_msgs/String "{data: "stop"}"
'''

if __name__ == '__main__':
    ###############
    try:
        [robot_motion, init_pose, robot_action, robot_task] = robot_model
        planner(robot_motion, init_pose, robot_action, robot_task)
    except rospy.ROSInterruptException:
        pass
