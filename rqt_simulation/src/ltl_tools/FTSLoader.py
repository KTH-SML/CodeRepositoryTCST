from ltl_tools.ts import MotionFts, ActionModel, MotActModel
#from ltl_tools.planner import ltl_planner

import yaml
import codecs
from math import radians, cos, sin
import time
import rospkg
import os

class FTSLoader(object):
    def __init__(self, _file):
        self.file = _file
        print(_file)
        stream = file(self.file, 'r')
        data_file = yaml.load(stream)
        FTS = data_file['FTS']
        _map = data_file['Map']
        stream.close()

        regions = {}
        ap = set()

        ##############################
        # motion FTS

        for i in range(0, len(FTS)):
            #ap.update({FTS.keys()[i]})
            for j in range(0, len(FTS[FTS.keys()[i]]['propos'])):
                if FTS[FTS.keys()[i]]['propos'][j] not in ap:
                    ap.update({FTS[FTS.keys()[i]]['propos'][j]})
            regions.update({(tuple(FTS[FTS.keys()[i]]['pose']['position']), tuple(FTS[FTS.keys()[i]]['pose']['orientation'])): set(FTS[FTS.keys()[i]]['propos'])})

        init_pose = ((7.77,  7.00, 0.0), (0.0, 0.0, 0.0, 1.0))

        robot_motion = MotionFts(regions, ap, _map)
        robot_motion.set_initial(init_pose)

        for i in range(0, len(FTS)):
            for j in range(0, len(FTS[FTS.keys()[i]]['edges'])):
                robot_motion.add_un_edges([[(tuple(FTS[FTS.keys()[i]]['pose']['position']), tuple(FTS[FTS.keys()[i]]['pose']['orientation'])), (tuple(FTS[FTS[FTS.keys()[i]]['edges'][j]['target']]['pose']['position']), tuple(FTS[FTS[FTS.keys()[i]]['edges'][j]['target']]['pose']['orientation']))]], unit_cost = FTS[FTS.keys()[i]]['edges'][j]['cost'])

        ##############################
        # action FTS

        ############# no action model
        # action = dict()
        ############# with action
        # for supported actions in play_motion
        # see http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion##############################

        action = {}
        robot_action = ActionModel(action)

        ##############################
        # specify tasks

        robot_hard_task = ''
        robot_soft_task = ''

        robot_task = [robot_hard_task, robot_soft_task]

        self.robot_model = [robot_motion, init_pose, robot_action, robot_task]

        ##############################
        # complete robot model
        # robot_full_model = MotActModel(robot_motion, robot_action)



        ##############################
        # specify tasks
        ########## only hard
        # hard_task = '<>(r1 && <> (r2 && <> r6)) && (<>[] r6)'
        #hard_task = '(<>(pick && <> drop)) && ([]<> r3) && ([]<> r6)'
        #soft_task = None


        ########## soft and hard
        # hard_task = '(([]<> r3) && ([]<> r4))'
        # soft_task = '([]! b)'
