from discrete_plan import dijkstra_targets
from product import ProdAut_Run
import rospy
from math import sqrt
from copy import deepcopy


class temporaryTask(object):
    def __init__(self):
        print('init')
        self.temporary_tasks = []
        self.final_propos = []
        self.task_end_index = []
        self.task_end_planner_index = []
        self.task_time = []
        self.propos_planner_index = []
        self.chosen_comb = []
        self.combinations = []

    def add_task(self, temporary_task):
        print(self.temporary_tasks)
        task_sequence = []
        for i in range(0, len(temporary_task.task)):
            task_sequence.append(temporary_task.task[i].data)
        self.final_propos.append(task_sequence[-1])
        self.temporary_tasks.append(task_sequence)
        self.task_time.append((rospy.Time.now(), temporary_task.T_des.data))

    def remove_task(self, planner_index):
        index = self.task_end_planner_index.index(planner_index)
        del self.final_propos[self.task_end_index[index]]
        del self.task_time[self.task_end_index[index]]
        for i in range(len(self.task_end_index)):
            if self.task_end_index[i] > self.task_end_index[index]:
                self.task_end_index[i] = self.task_end_index[i] - 1
        del self.task_end_index[self.task_end_index[index]]
        del self.task_end_planner_index[index]

    def remove_propos(self):
        for i in range(0, len(self.combinations)):
            print(self.chosen_comb[0])
            print(self.combinations[i])
            index = self.combinations[i].index(self.chosen_comb[0])
            del self.combinations[i][index]
            counter = 0
        for i in range(0, len(self.temporary_tasks)):
            if self.temporary_tasks[i-counter][0] == self.chosen_comb[0]:
                del self.temporary_tasks[i][0]
                if len(self.temporary_tasks[i-counter]) == 0:
                    del self.temporary_tasks[i-counter]
                    counter = counter + 1
        del self.chosen_comb[0]

    def make_combination_set(self):
        propositions = []
        for i in range(0, len(self.temporary_tasks)):
            for j in range(0, len(self.temporary_tasks[i])):
                if self.temporary_tasks[i][j] not in propositions:
                    propositions.append(self.temporary_tasks[i][j])
        combinations = list(self.permutations(propositions))
        self.combinations = self.check_sequences(combinations)
        return self.combinations

    def permutations(self, propositions):
        if len(propositions) <= 1:
            yield propositions
        else:
            for i in range(0, len(propositions)):
                for perm in self.permutations(propositions[:i] + propositions[i+1:]):
                    yield [propositions[i]] + perm

    def check_sequences(self, combinations):
        invalid_comb_index = []
        for i in range(0, len(combinations)):
            for j in range(0, len(self.temporary_tasks)):
                for m in range(0, len(self.temporary_tasks[j])-1):
                    index = combinations[i].index(self.temporary_tasks[j][m])
                    if index > combinations[i].index(self.temporary_tasks[j][m+1]) and i not in invalid_comb_index:
                        invalid_comb_index.append(i)
        #print(invalid_comb_index)
        for i in range(0, len(invalid_comb_index)):
            del combinations[invalid_comb_index[i]-i]
        return combinations

    def inverse_labeling_function(self, proposition, ts):
        poses = []
        for node in ts.nodes():
            #print(ts.node[node]['label'])
            if proposition in ts.node[node]['label']:
                poses.append(node)
        return poses

    def build_target_set(self, proposition, product):
        ts_nodes = self.inverse_labeling_function(proposition, product.graph['ts'])
        #print(ts_nodes)
        target_set = []
        for i in range(0, len(ts_nodes)):
            for buchi_node in product.graph['buchi'].nodes():
                prod_node = product.composition(ts_nodes[i], buchi_node)
                target_set.append(prod_node)
        #print(target_set)
        return target_set

    def find_temporary_run(self, current_state, product):
        cost = float('inf')
        for i in range(0, len(self.combinations)):
            run_prefix = []
            cost_temp = 0
            cost_with_factor = 0
            distance = 0
            propos_planner_index = []
            task_end_index = []
            task_end_planner_index = []
            #comb = []
            for j in range(0, len(self.combinations[i])):
                target_set = self.build_target_set(self.combinations[i][j], product)
                runs = {}
                if j == 0:
                    for (prefix, precost) in dijkstra_targets(product, current_state, target_set):
                        runs[(prefix[0], prefix[-1])] = (prefix, precost)
                        #print(prefix)
                else:
                    for (prefix, precost) in dijkstra_targets(product, run_prefix[-1], target_set):
                        runs[(prefix[0], prefix[-1])] = (prefix, precost)
                prefix_temp, cost_ = min(runs.values(), key = lambda p: p[1])
                run_prefix = run_prefix + prefix_temp
                cost_temp = cost_temp + cost_
                for m in range(1, len(prefix_temp)):
                    #print(prefix_temp[m-1][0][0][0])
                    distance = distance + self.euclidean_distance(prefix_temp[m-1][0][0][0], prefix_temp[m][0][0][0])
                    #print('---distance---')
                    #print(distance)

                #print(self.combinations[i][j])

                propos_planner_index.append(len(run_prefix) - 1)
                #comb.append([self.combinations[i][j]])

                if self.combinations[i][j] in self.final_propos:
                    task_index = self.final_propos.index(self.combinations[i][j])
                    planner_index = len(run_prefix) - 1
                    task_end_index.append(task_index)
                    task_end_planner_index.append(planner_index)
                    cost_with_factor = cost_with_factor + cost_temp*(distance/0.3 + (rospy.Time.now()-self.task_time[task_index][0]).to_sec())/self.task_time[task_index][1]
            print('---costwithfactor---')
            print(cost_with_factor)
            print(cost)
            if cost_with_factor < cost:
                cost = cost_with_factor
                prefix_ = run_prefix
                self.chosen_comb = deepcopy(self.combinations[i])
                self.propos_planner_index = propos_planner_index
                self.task_end_index = task_end_index
                self.task_end_planner_index = task_end_planner_index
                print('---run_temp---')
        print(prefix_[0])
        run_temp = ProdAut_Run(product, prefix_, cost, [], [], cost_temp)
        return run_temp

    def euclidean_distance(self, position1, position2):
        return (sqrt((position1[0]-position2[0])**2+(position1[1]-position2[1])**2+(position1[2]-position2[2])**2))
