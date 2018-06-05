# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history, validate_and_revise_after_sense_info, validate_run_and_revise


class ltl_planner(object):
	def __init__(self, ts, hard_spec, soft_spec):
		buchi = mission_to_buchi(hard_spec, soft_spec)
		self.product = ProdAut(ts, buchi)
		self.Time = 0
		self.cur_pose = None
		self.trace = [] # record the regions been visited
		self.run_history = [] # record executed plan
		self.traj = [] # record the full trajectory
		self.opt_log = []
		# record [(time, prefix, suffix, prefix_cost, suffix_cost, total_cost)]
		self.com_log = []
		# record [(time, no_messages)]
		self.num_changed_regs = 0

	def optimal(self, beta=10, style='static'):
		self.beta = beta
		if style == 'static':
			# full graph construction
			self.product.graph['ts'].build_full()
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
		elif style == 'ready':
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
		elif style == 'on-the-fly':
			# on-the-fly construction
			self.product.build_initial()
			self.product.build_accept()
			self.run, plantime = dijkstra_plan_optimal(self.product, self.beta)
                if self.run == None:
                        print '---No valid has been found!---'
                        print '---Check you FTS or task---'
                        return
		#print '\n'
                print '------------------------------'
                print 'the prefix of plan **states**:'
		print [n for n in self.run.line]
                print 'the suffix of plan **states**:'
		print [n for n in self.run.loop]
		#print '\n'
                print '------------------------------'
		print 'the prefix of plan **actions**:'
		print [n for n in self.run.pre_plan]
		print 'the suffix of plan **actions**:'
		print [n for n in self.run.suf_plan]
		self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
		self.index = 0
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
		return plantime

	def find_next_move(self):
		print('---len sufplan---')
		print(len(self.run.suf_plan))
		if self.segment == 'line' and self.index < (len(self.run.pre_plan)-2):
			self.trace.append(self.run.line[self.index])
			self.run_history.append(self.run.pre_plan[self.index])
			self.index += 1
			self.next_move = self.run.pre_plan[self.index]
		elif (self.segment == 'line') and ((self.index >= len(self.run.pre_plan)-2) or (len(self.run.pre_plan) <= 2)):
			self.trace.append(self.run.line[self.index])
			self.run_history.append(self.run.pre_plan[self.index])
			self.index = 0
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-1:
			self.trace.append(self.run.loop[self.index])
			self.run_history.append(self.run.suf_plan[self.index])
			self.index += 1
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		elif (self.segment == 'loop') and ((self.index >= len(self.run.suf_plan)-1) or (len(self.run.suf_plan) <= 2)):
			self.trace.append(self.run.loop[self.index])
			self.run_history.append(self.run.suf_plan[self.index])
			self.index = 1
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		print('---trace---')
		print(len(self.trace))
		return self.next_move


	def update_add(self, object_name, region_label):
		MotionFts = self.product.graph['ts'].graph['region']
		for ts_node in MotionFts.nodes():
			if region_label in MotionFts.node[ts_node]['label']:
				MotionFts.node[ts_node]['label'].add(object_name)

	def update_remove(self, object_name, region_label):
		MotionFts = self.product.graph['ts'].graph['region']
		for ts_node in MotionFts.nodes():
			if region_label in MotionFts.node[ts_node]['label']:
                                if object_name in MotionFts.node[ts_node]['label']:
                                        MotionFts.node[ts_node]['label'].remove(object_name)
                                        return True
                                else:
                                        return False

	def update_knowledge(self, sense_info):
		changed_regs = self.product.graph['ts'].graph['region'].update_ts_after_sense_info(sense_info)
		if changed_regs:
			changed_states = self.product.update_prod_aut_after_ts_update(sense_info)
			self.num_changed_regs = self.num_changed_regs + len(changed_regs)


	def replan(self):
		new_run = improve_plan_given_history(self.product, self.trace, self.run, self.index)

		if new_run:
			self.run = new_run
			print('---after replan---')
			print(self.run.pre_plan)
			self.index = 0
			self.segment = 'line'
			self.trace = []
			self.run_history = []
			#self.find_next_move()
			#self.next_move = self.run.pre_plan[self.index]

	def validate_and_revise(self):
		validate_run_and_revise(self.product, self.run)




	def replan_simple(self, pose):
		self.product.graph['ts'].graph['region'].set_initial(pose)
		self.optimal(10, style='static')
                self.index = 0
		self.segment = 'loop'
		self.next_move = self.run.suf_plan[self.index]
