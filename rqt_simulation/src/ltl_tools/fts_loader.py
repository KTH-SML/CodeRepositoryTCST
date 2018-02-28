from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner

import yaml
import codecs
from math import radians, cos, sin
import time
import rospkg
import os


def compute_poly(center, angle, width, length):
    norm_poly = [[center[0]+s[0]*0.5*width,center[1]+s[1]*0.5*length] for s in [[-1,0],[0,1],[1,0],[0,-1]]]
    target_poly = []
    for p in norm_poly:
        rot_p = rotate_around_center(p, center, angle)
        target_poly.append(rot_p)
    return target_poly

def rotate_around_center(p, c, angle):
    x, y = p
    c_x, c_y = c
    moved_x = x - c_x
    moved_y = y - c_y
    rotated_x, rotated_y = rotate((moved_x, moved_y), angle)
    back_x = rotated_x + c_x
    back_y = rotated_y + c_y
    return back_x, back_y

def rotate(p,angle):
    x, y = p
    rad = radians(angle)
    x_new = cos(rad) * x - sin(rad) * y
    y_new = sin(rad) * x + cos(rad) * y
    return x_new, y_new


##############################
# motion FTS
ap = {'r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r7','r8', }

regions = {   (10.36, -6.19, 0.0): set(['r1',]),
              ( 8.07, -0.733, 0.0): set(['r2',]),
              ( 4.55, -6.38, 0.0): set(['r3',]),
              (-0.61, -7.42, 0.0): set(['r4',]),
              (-5.69, -8.53, 0.0): set(['r5',]),
              (-0.65, -0.61, 0.0): set(['r6',]),
              (-9.63, -1.12, 0.0): set(['r7',]),
              (7.77,  7.00, 0.0): set(['r8',]),
}

#print(regions)
#print(ap)

env_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'FTS', 'env_GUI.yaml')
stream = file(env_file, 'r')
data = yaml.load(stream)
stream.close()
test = {}
test_ap = set()

#print(data)
for i in range(0, len(data)):
    test_ap.update({data.keys()[i]})
    #test.update( {tuple(data[data.keys()[i]]['position']): set([data.keys()[i]])})
    test.update( {(tuple(data[data.keys()[i]]['pose']['position']), tuple(data[data.keys()[i]]['pose']['orientation'])): set([data.keys()[i]])})

print('test')
print(test)
print(test_ap)

init_pose = ((7.77,  7.00, 0.0), (0.0, 0.0, 0.0, 1.0))
#robot_motion = MotionFts(regions, ap, 'pal_office' )
robot_motion = MotionFts(test, test_ap, 'pal_office' )
robot_motion.set_initial(init_pose)
#robot_motion.add_full_edges(unit_cost = 0.1)

for i in range(0, len(data)):
    for j in range(0, len(data[data.keys()[i]]['edges'])):
        print(data[data.keys()[i]]['edges'][j]['cost'])
        #robot_motion.add_un_edges([[tuple(data[data.keys()[i]]['position']), tuple(data[data[data.keys()[i]]['edges'][j]['target']]['position'])]], unit_cost = data[data.keys()[i]]['edges'][j]['cost'])
        robot_motion.add_un_edges([[(tuple(data[data.keys()[i]]['pose']['position']), tuple(data[data.keys()[i]]['pose']['orientation'])), (tuple(data[data[data.keys()[i]]['edges'][j]['target']]['pose']['position']), tuple(data[data[data.keys()[i]]['edges'][j]['target']]['pose']['orientation']))]], unit_cost = data[data.keys()[i]]['edges'][j]['cost'])


##############################
# action FTS

############# no action model
# action = dict()
############# with action
# for supported actions in play_motion
# see http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion

action = {'col_grasp': (10, 'hm', set(['col_grasp',])),}
robot_action = ActionModel(action)

##############################
# specify tasks

task_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'config', 'task', 'task.yaml')
stream = file(task_file, 'r')
data = yaml.load(stream)
stream.close()
robot_hard_task = data['hard_task']
robot_soft_task = data['soft_task']

#robot_hard_task = '([]<> R01) && ([]<> R02) && ([]<> R03)'

#robot_hard_task = task
#robot_soft_task = ''
robot_task = [robot_hard_task, robot_soft_task]
print(robot_task)




robot_model = [robot_motion, init_pose, robot_action, robot_task]
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
