#!/usr/bin/env python

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
from Plotter import Plotter

scenario = 'hotel'
trajectory_files = (
                    '/home/lukas/catin_ws/src/rqt_simulation_gui/rqt_simulation/logging/robot1_trajectory_2018-07-05-10-34.txt',
                    )
task_files = (
                '/home/lukas/catin_ws/src/rqt_simulation_gui/rqt_simulation/logging/task_times.txt'
                )
FTS_file = '/home/lukas/catin_ws/src/rqt_simulation_gui/rqt_simulation/config/FTS/hotel_scenario.yaml'

plotter = Plotter(scenario, trajectory_files)

imgplot = plt.imshow(plotter.map)
imgplot.set_cmap('hot')
imgplot.axes.get_xaxis().set_visible(False)
imgplot.axes.get_yaxis().set_visible(False)
plotter.load_FTS(FTS_file)

plt.plot(plotter.FTS_u, plotter.FTS_v, 'o', markersize=30, c='r')

for i in range(0, len(plotter.FTS_labels)):
    plt.text(plotter.FTS_u[i], plotter.FTS_v[i], plotter.FTS_labels[i], fontsize=20)



if task_files:
    task_data = np.loadtxt(task_files, delimiter="\t")
    task_times = [task_data,]

print(task_times)

colors = ['b', 'b', 'c', 'g', 'm']
counter = 0
index = 0

for i in range(0, len(trajectory_files)):
    plotter.load_traj(trajectory_files[i])
    for j in range(0, len(plotter.u)):
        if counter == len(task_times):
            plt.plot(plotter.u[index:], plotter.v[index:], c=colors[counter], linewidth=3)

            break
        if plotter.time[j] > task_times[counter]:

        #time_rounded = [round(elem) for elem in plotter.time]
        #index = time_rounded.index(task_times[j])
            plt.plot(plotter.u[index:j+1], plotter.v[index:j+1], c=colors[counter], linewidth=3)
            index = j
            counter = counter + 1
            if counter == 2:
                plt.show()




plt.show()
