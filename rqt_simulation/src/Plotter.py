#!/usr/bin/env python

import os
import rospkg
import yaml
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Plotter():
    def __init__(self, scenario, trajectory_files):
        self.scenario = scenario
        self.trajectory_files = trajectory_files
        self.load_map(self.scenario)
        print(self.trajectory_files)
        #self.load_traj(self.trajectory_files[0])

    def load_map(self, scenario):
        self.scenario = scenario
        map_yaml = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'scenarios', scenario, 'map.yaml')
        self.loadConfig(map_yaml)
        map = 'map.png'

        map_file = os.path.join(rospkg.RosPack().get_path('rqt_simulation'), 'scenarios', scenario, map)

        self.map = plt.imread(map_file)
        map_height = len(self.map)
        self.worldOrigin = (-self.map_origin[0]/self.map_resolution, self.map_origin[1]/self.map_resolution + map_height)

    def loadConfig(self, filename):
        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()
        self.map_image = data['image']
        self.map_resolution = data['resolution']
        self.map_origin = tuple(data['origin'])
        self.map_negate = data['negate']
        self.map_occupied_thresh = data['occupied_thresh']
        self.map_free_thresh = data['free_thresh']

    def load_traj(self, trajectory_file):
        d = np.loadtxt(trajectory_file, delimiter="\t")

        time = []
        self.u = []
        self.v = []

        for i in range(0, len(d)):
            time.append(d[i][0])
            pixel_coords_u = d[i][1] / self.map_resolution + self.worldOrigin[0]
            pixel_coords_v = -d[i][2] / self.map_resolution + self.worldOrigin[1]
            self.u.append(pixel_coords_u)
            self.v.append(pixel_coords_v)

    def load_FTS(self, FTS_file):
            # Start file dialog
            stream = file(FTS_file, 'r')
            data = yaml.load(stream)
            stream.close()

            self.FTS_u = []
            self.FTS_v = []
            self.FTS_labels = []
            # Load FTS
            self.region_of_interest = {}
            self.region_of_interest = data['FTS']
            for i in range(0, len(self.region_of_interest.keys())):
                pixel_coords_u = self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position'][0] / self.map_resolution + self.worldOrigin[0]
                pixel_coords_v = -self.region_of_interest[self.region_of_interest.keys()[i]]['pose']['position'][1] / self.map_resolution + self.worldOrigin[1]
                self.FTS_u.append(pixel_coords_u)
                self.FTS_v.append(pixel_coords_v)
                self.FTS_labels.append(self.region_of_interest.keys()[i])
