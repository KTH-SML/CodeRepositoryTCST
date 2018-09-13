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
		self.gamma = []
		self.r = []
		self.rho_max = []
		self.t = []
		self.t_rel_t0 = []
		self.t_rel_tr = []
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
		self.rho_max.append(msg.rho_max)
		self.r.append(msg.r)
		self.gamma.append(msg.gamma)
		self.t_rel_t0.append(msg.t_relative_t0)
		self.t_rel_tr.append(msg.t_relative_tr)

	def plotRobustness(self):
		t = np.array(self.t)
		tt = t - t[0]
 
		gamma = np.array(self.gamma)
		r = np.array(self.r)
		rho_max = np.array(self.rho_max)

		g = np.zeros((tt.shape[0],))

		fig = plt.figure()
		ax = plt.axes()
		ax.minorticks_on()
		ax.grid(which='major')
		ax.grid(which='minor', alpha=0.4)
		ax.set_ylim([-2, 0.6])
		ax.set_xlabel("t [s]")
		plt.plot(tt, self.robustness, color='black', linewidth='1.5', label='${{\\rho}}^{{\\psi_{{{0}}}}}(\\mathbf{{x}}_{{\\phi_{{{0}}}}}(t))$'.format(self.robot))
		plt.plot(tt, -gamma+rho_max, color='red', linewidth='1.5', label='upper and lower funnel')
		plt.plot(tt, r, color='red', label='$r_{{{0}}}$'.format(self.robot), linestyle=':', linewidth='1.5')
		plt.plot(tt, rho_max, color='red', linewidth='1.5')
		plt.legend(loc=4, fontsize='large')
		fig.savefig(os.path.dirname(os.path.dirname(__file__))+'/rho'+str(self.robot)+'.png')
		


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

