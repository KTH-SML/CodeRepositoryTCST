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
                    '/home/lukas/catin_ws/src/rqt_simulation_gui/rqt_simulation/logging/robot1_trajectory_2018-07-03-09-38.txt',
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

for i in range(0, len(trajectory_files)):
    plotter.load_traj(trajectory_files[i])
    plt.plot(plotter.u, plotter.v)


plt.show()
