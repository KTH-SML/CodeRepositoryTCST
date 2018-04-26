#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
from hybrid_controller.msg import CriticalEvent
from hybrid_controller.msg import Robustness
from geometry_msgs.msg import Twist
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
		self.u = []
		self.uppc = []
		self.upfc = []

		self.critical_event_sub = rospy.Subscriber("/critical_event"+str(self.robot), CriticalEvent, self.criticalEventCallback)
		self.robustness_sub = rospy.Subscriber("/robustness"+str(self.robot), Robustness, self.robustnessCallback)
		self.u_sub = rospy.Subscriber("/cmdvel"+str(self.robot), Twist, self.uCallback)
		self.uppc_sub = rospy.Subscriber("/uppc"+str(self.robot), Twist, self.uppcCallback)
		self.upfc_sub = rospy.Subscriber("/upfc"+str(self.robot), Twist, self.upfcCallback)

	def uppcCallback(self, msg):
		self.uppc.append(msg)

	def upfcCallback(self, msg):
		self.upfc.append(msg)

	def uCallback(self, msg):
		self.u.append(msg)

	def criticalEventCallback(self, msg):
		self.critical_events.append(msg)
		self.t_ce.append(msg.stamp.to_sec())

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
		g = np.zeros((tt.shape[0],))

		tc = np.array(self.t_ce)
		c = self.critical_events[0]
		for i in range(tc.shape[0]):
			tc[i] = t_rel[np.where(t>self.t_ce[i])[0][0]]

		gamma[t_rel<=tc[0]] = -(c.gamma_0[0]-c.gamma_inf[0])*np.exp(-c.l[0]*t_rel[t_rel<=tc[0]])-c.gamma_inf[0]+c.rho_max[0]
		r[t_rel<=tc[0]] = c.r[0]
		rho_max[t_rel<=tc[0]] = c.rho_max[0]
		g[t_rel<=tc[0]] = (c.gamma_0[0]-c.gamma_inf[0])*np.exp(-c.l[0]*t_rel[t_rel<=tc[0]])+c.gamma_inf[0]
		for i in range(tc.shape[0]):
			c = self.critical_events[i]
			gamma[t_rel>tc[i]] = -(c.gamma_0[1]-c.gamma_inf[1])*np.exp(-c.l[1]*t_rel[t_rel>tc[i]])-c.gamma_inf[1]+c.rho_max[1]
			r[t_rel>tc[i]] = c.r[1]
			rho_max[t_rel>tc[i]] = c.rho_max[1]
			g[t_rel>tc[i]] = (c.gamma_0[1]-c.gamma_inf[1])*np.exp(-c.l[1]*t_rel[t_rel>tc[i]])+c.gamma_inf[1]

		fig = plt.figure()
		plt.plot(t_rel, self.robustness, color='blue')

		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		ax.set_ylim([-3, 0.6])
		
		#for c in self.critical_events:
		#	plt.plot(t_rel, -(c.gamma_0[0]-c.gamma_inf[0])*np.exp(-c.l[0]*t_rel)-c.gamma_inf[0]+c.rho_max[0], color='black', linestyle='dashed', linewidth='0.5')
		#plt.plot(t_rel, -(c.gamma_0[1]-c.gamma_inf[1])*np.exp(-c.l[1]*t_rel)-c.gamma_inf[1]+c.rho_max[1], color='black', linestyle='dashed', linewidth='0.5')

		plt.plot(t_rel, gamma, color='green')
		plt.plot(t_rel, r, color='red')
		plt.plot(t_rel, rho_max, color='green')
		fig.savefig(os.path.dirname(os.path.dirname(__file__))+'/rho'+str(self.robot)+'.png')

		fig2 = plt.figure()
		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		vx = np.array(list(u.linear.x for u in self.u))
		vy = np.array(list(u.linear.y for u in self.u))
		vx2 = np.square(vx)
		vy2 = np.square(vy)
		v = np.sqrt(vx2+vy2)
		plt.plot(t_rel, v)
		fig2.savefig(os.path.dirname(os.path.dirname(__file__))+'/u'+str(self.robot)+'.png')

		rho = np.array(self.robustness)
		e = (rho - rho_max)/g
		fig3 = plt.figure()
		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		plt.plot(t_rel, e)
		fig3.savefig(os.path.dirname(os.path.dirname(__file__))+'/e'+str(self.robot)+'.png')

		uppcx = np.array(list(u.linear.x for u in self.uppc))
		uppcy = np.array(list(u.linear.y for u in self.uppc))
		upfcx = np.array(list(u.linear.x for u in self.upfc))
		upfcy = np.array(list(u.linear.y for u in self.upfc))
		uppc = np.sqrt(np.square(uppcx)+np.square(uppcy))
		upfc = np.sqrt(np.square(upfcx)+np.square(upfcy))
		uppcdir = np.arctan2(uppcy, uppcx)
		upfcdir = np.arctan2(upfcy, upfcx)
		plotSignal(t_rel, uppc, self.robot, 'uppc')
		plotSignal(t_rel, upfc, self.robot, 'upfc')
		plotSignal(t_rel, uppcdir, self.robot, 'uppcdir')
		plotSignal(t_rel, upfcdir, self.robot, 'upfcdir')
		


def plotSignal(t, x, id, name):
	fig = plt.figure()
	ax = plt.axes()
	ax.minorticks_on()
	ax.grid(which='major')
	ax.grid(which='minor', alpha=0.4)
	plt.plot(t, x)
	fig.savefig(os.path.dirname(os.path.dirname(__file__))+'/'+name+str(id)+'.png')


if __name__ == '__main__':
	p = Plotter()
	
	rospy.spin()

