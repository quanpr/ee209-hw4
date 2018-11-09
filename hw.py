import pdb
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
import random
from copy import deepcopy

default_map = [[255]*512 for i in range(512)]
sample1_map = deepcopy(default_map)
sample2_map = deepcopy(default_map)
for i in range(100, 200):
	default_map[i][200:500] = [0]*(500-200)
for i in range(400,500):
	default_map[i][100:300] = [0]*(300-100)
for j in range(200, 400):
	default_map[j][200:300] = [125]*(300-200)

for i in range(50, 150):
	sample2_map[i][100:400] = [0]*(400-100)
for i in range(150, 200):
	sample2_map[i][100:400] = [125]*(400-100)
for i in range(200,250):
	sample2_map[i][100:400] = [0]*(400-100)
for i in range(250, 300):
	sample2_map[i][100:400] = [75]*(400-100)
for i in range(300, 350):
	sample2_map[i][100:400] = [0]*(400-100)
for i in range(350, 400):
	sample2_map[i][100:350] = [0]*(350-100)

for i in range(100, 200):
	sample1_map[i][100:200] = [0]*(200-100)
	sample1_map[i][300:400] = [0]*(400-300)
for i in range(300,400):
	sample1_map[i][100:200] = [0]*(200-100)
	sample1_map[i][300:400] = [0]*(400-300)
for i in range(400, 512):
	sample1_map[i][100:200] = [0]*(200-100)
	sample1_map[i][300:400] = [0]*(400-300)

class robot:
	def __init__(self, initial_idx, goal_idx, Da=8, o_space=default_map):
		self.initial_idx = initial_idx
		self.goal_idx = goal_idx
		self.Da = Da
		self.o_space = o_space
		self.max_fail = 10**4
		self.found_path = False
		# each node: 'idx':(x, y, theta), 'parent' = parent number, 'distance' = d
		self.RRTree = []
		self.Dx = len(o_space)
		self.Dy = len(o_space[0])
		#self.theta_range = 18
		self.bias = 0.8
		self.threshold = 10

		self.RRTree.append({'idx':self.initial_idx, 'parent':None,
							'distance':0})

	def determine_angle(self, start, end):
		dx, dy = end[0]-start[0], end[1]-start[1]
		if dx == 0:
			return np.pi/2 if dy > 0 else 3/2*np.pi
		theta = np.arctan(abs(dy/dx))
		if dx > 0 and dy >= 0:
			return theta
		elif dx < 0 and dy >= 0:
			return np.pi-theta
		elif dx < 0 and dy < 0:
			return theta + np.pi
		else:
			return 2*np.pi-theta

	def generate_new_path(self, start, end):
		result = []
		dx, dy = end[0]-start[0], end[1]-start[1]
		if dx == 0:
			result = [(start[0], start[1]+i, end[2]) for i in range(1,dy+1)]
		else:
			for i in range(1, abs(dx)+1):
				node = (start[0]+int(i*abs(dx)/dx), start[1]+int(i*dy/abs(dx)), end[2])
				result.append(node)
		return result

	def check_node(self, node):
		if self.o_space[node[0]][node[1]] == 255:
			return True
		elif self.o_space[node[0]][node[1]] == 125 and (0<=node[2]<=np.pi):
			return True
		elif self.o_space[node[0]][node[1]] == 75 and (np.pi<=node[2]<=2*np.pi):
			return True
		else:
			return False

	def valiadate_new_node(self, start, end):
		path = self.generate_new_path(start, end)
		return all([self.check_node(node) for node in path])

	def nearest_k_neighbor(self, target_idx, num_of_neighbor=1):
		def distance(start, end):
			#return (1/(sum(len(start)))*sum([(start[i]-end[i])**2 for i in range(len(start))]))**0.5
			return ((start[0]-end[0])**2+(start[1]-end[1])**2)**0.5
		if num_of_neighbor == 1:
			# j: distance, i: correpsonding index
			d = [(j, i) for i, j in enumerate([distance(target_idx, node_idx['idx']) for node_idx in self.RRTree])]
			nearest =  min(d, key=lambda x:x[0])
			return nearest[1]
		else:
			# improve KNN sorting with heap
			import heapq
			# j: distance, i: correpsonding index
			d = [(j, i) for i, j in enumerate([distance(target_idx, node_idx['idx']) for node_idx in self.RRTree])]
			result = []
			for idx in d:
				if len(result) < num_of_neighbor:
					heapq.heappush(result, idx)
				else:
					heap.heappushpop(result, idx)
			return [node[1] for node in result]

	def generate_next_move(self, start, rand):
		dx, dy = rand[0]-start[0], rand[1]-start[1]
		if dx == 0:
			return (start[0], start[1] + self.Da, rand[2])
		else:
			theta = np.arctan(abs(dy/dx))
			x = start[0] + int(self.Da*np.cos(theta)) if dx > 0 else int(start[0] - self.Da*np.cos(theta))
			y = start[1] + int(self.Da*np.sin(theta)) if dy > 0 else int(start[1] - self.Da*np.sin(theta))
			return (max(0, min(x, self.Dx)), max(0, min(y, self.Dy)), rand[2])

	def plot_tree(self):
		plt.clf()
		#plt.figure()
		plt.imshow(self.o_space, cmap='gray')

		for node in self.RRTree:
			if node['parent'] is not None:
				plt.plot([node['idx'][1], self.RRTree[node['parent']]['idx'][1]],
						[node['idx'][0], self.RRTree[node['parent']]['idx'][0]], '-g')

		plt.plot(self.initial_idx[1], self.initial_idx[0], 'xr')
		plt.plot(self.goal_idx[1], self.goal_idx[0], 'xr')
		plt.grid()
		#plt.show()
		plt.pause(0.01)

	def planning(self):

		def distance(start, end):
			return ((start[0]-end[0])**2+(start[1]-end[1])**2)**0.5

		fail_attempt = 0
		while fail_attempt <= self.max_fail and not self.found_path:
			#random.seed(0)
			if random.uniform(0,1) < self.bias:
				rand_x, rand_y = random.randint(0, self.Dx-1), random.randint(0, self.Dy-1)
			else:
				rand_x, rand_y = self.goal_idx[0], self.goal_idx[1]

			# return the nearest idx
			nearest_idx = self.nearest_k_neighbor((rand_x, rand_y, 0), 1)
			# position of the nearest idx
			idx_position = self.RRTree[nearest_idx]['idx']
			rand = (rand_x, rand_y, self.determine_angle(idx_position, (rand_x, rand_y, 0)))

			new_move = self.generate_next_move(idx_position, rand)
			#pdb.set_trace()
			if not self.valiadate_new_node(idx_position, new_move):
				fail_attempt += 1
				continue

			#check if pre exist in the tree
			idx_in_tree = self.nearest_k_neighbor(idx_position, 1)
			node_in_tree = self.RRTree[idx_in_tree]['idx']
			if distance(node_in_tree, new_move) < self.threshold:
				# fail attempt
				fail_attempt += 1
				continue


			new_move_distance = self.RRTree[nearest_idx]['distance'] + distance(new_move, idx_position)
			self.RRTree.append({'idx': new_move,
								'parent':nearest_idx,
								'distance':new_move_distance})

			self.plot_tree()

			# check whether reach goal
			distance_to_goal = distance(new_move, self.goal_idx)
			if distance_to_goal < self.threshold and \
				self.valiadate_new_node(new_move, self.goal_idx):
				self.found_path = True
				self.RRTree.append({'idx': self.goal_idx,
								'parent':len(self.RRTree)-1,
								'distance':0})
				self.plot_tree()
				break


if __name__ == '__main__':
	initial_idx, goal_idx = (10, 20, np.pi), (375, 375, 0)
	robot = robot(initial_idx, goal_idx, 15,sample2_map)
	#print(robot.generate_new_path(initial_idx, goal_idx))
	s = robot.generate_new_path((150,198,3.14159),(139,215,2.11))
	#pdb.set_trace()
	robot.planning()
	pdb.set_trace()
