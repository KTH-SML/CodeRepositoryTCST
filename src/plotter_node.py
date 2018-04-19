#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
from hybrid_controller.msg import CriticalEvent
from hybrid_controller.msg import Robustness
import os


class Plotter:
	def __init__(self):
		rospy.init_node("plotter")
		rospy.on_shutdown(self.plotRobustness)
		self.critical_events = []
		self.t_ce = []
		self.robustness = []
		self.t = []
		self.t_rel = []
		self.robot = rospy.get_param('~robot')

		self.critical_event_sub = rospy.Subscriber("/critical_event"+str(self.robot), CriticalEvent, self.criticalEventCallback)
		self.robustness_sub = rospy.Subscriber("/robustness"+str(self.robot), Robustness, self.robustnessCallback)

	def criticalEventCallback(self, msg):
		self.critical_events.append(msg)
		self.t_ce.append(msg.stamp.to_sec())
		print msg.t_star, msg.r, msg.rho_max, msg.gamma_0, msg.gamma_inf, msg.l

	def robustnessCallback(self, msg):
		self.robustness.append(msg.rho)
		self.t.append(msg.stamp.to_sec())
		self.t_rel.append(msg.t_relative)

	def plotRobustness(self):
		t = np.array(self.t)
		tt = t - t[0]

		gamma = np.zeros((tt.shape[0],))
		r = np.zeros((tt.shape[0],))
		rho_max = np.zeros((tt.shape[0],))
		t_rel = np.array(self.t_rel)

		tc = np.array(self.t_ce)
		c = self.critical_events[0]
		for i in range(tc.shape[0]):
			tc[i] = t_rel[np.where(t>self.t_ce[i])[0][0]]

		gamma[t_rel<=tc[0]] = -(c.gamma_0[0]-c.gamma_inf[0])*np.exp(-c.l[0]*t_rel[t_rel<=tc[0]])-c.gamma_inf[0]+c.rho_max[0]
		r[t_rel<=tc[0]] = c.r[0]
		rho_max[t_rel<=tc[0]] = c.rho_max[0]
		for i in range(tc.shape[0]):
			c = self.critical_events[i]
			gamma[t_rel>tc[i]] = -(c.gamma_0[1]-c.gamma_inf[1])*np.exp(-c.l[1]*t_rel[t_rel>tc[i]])-c.gamma_inf[1]+c.rho_max[1]
			r[t_rel>tc[i]] = c.r[1]
			rho_max[t_rel>tc[i]] = c.rho_max[1]

		fig = plt.figure()
		plt.plot(t_rel, self.robustness, color='blue')

		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		
		for c in self.critical_events:
			plt.plot(t_rel, -(c.gamma_0[0]-c.gamma_inf[0])*np.exp(-c.l[0]*t_rel)-c.gamma_inf[0]+c.rho_max[0], color='black', linestyle='dashed', linewidth='0.5')
		plt.plot(t_rel, -(c.gamma_0[1]-c.gamma_inf[1])*np.exp(-c.l[1]*t_rel)-c.gamma_inf[1]+c.rho_max[1], color='black', linestyle='dashed', linewidth='0.5')

		plt.plot(t_rel, gamma, color='green')
		plt.plot(t_rel, r, color='red')
		plt.plot(t_rel, rho_max, color='green')
		fig.savefig(os.path.dirname(os.path.dirname(__file__))+'/rho'+str(self.robot)+'.png')


if __name__ == '__main__':
	p = Plotter()
	
	rospy.spin()

