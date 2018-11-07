import pdb
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math

default_map = [[255]*512 for i in range(512)]
for i in range(100, 200):
	default_map[i][200:500] = [0]*(500-200)
for i in range(400,500):
	default_map[i][100:300] = [0]*(300-100)
for j in range(200, 400):
	default_map[j][200:300] = [125]*(300-200)


class robot:
	def __init__(self, initial_idx, goal_idx, Da=8, e_space=default_map):
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
		self.theta_range = 18

	def generate_new_path(self, start, end):
		result = []
		dx, dy = end[0]-start[0], end[1]-start[1]
		if dx == 0:
			result = [(start[0], start[1]+i, start[2]) for i in range(1,dy+1)]
		else:
			for i in range(1, abs(dx)+1):
				node = (start[0]+i, start[1]+int(i*dy/dx), start[2])
				result.append(node)
		return result

	def valiadate_new_node(self, start, end):
		def check_node(node):
			if self.o_space[node[0]][node[1]] == 0:
				return True
			elif self.o_space[node[0]][node[1]] == 125 and np.pi<=node[2]<=2*np.pi:
				return True
			else:
				retrun False
		path = self.generate_new_path(start, end)
		return all([check_node(node) for node in path])

	def nearest_k_neighbor(self, target_idx, num_of_neighbor=1):
		def distance(start, end):
			#return (1/(sum(len(start)))*sum([(start[i]-end[i])**2 for i in range(len(start))]))**0.5
			return (1/2*((start[0]-end[0])**2+(start[1]-end[1])))**0.5
		if num_of_neighbor == 1:
			# j: distance, i: correpsonding index
			d = [(j, i) for i, j in enumerate([distance(target_idx, node_idx['idx']) for node_idx in self.RRTree])]
			nearest =  min(d, key=lambda x:x[0])[1]
			return self.RRTree[nearest]['idx']
		else:
			import heapq
			# j: distance, i: correpsonding index
			d = [(j, i) for i, j in enumerate([distance(target_idx, node_idx['idx']) for node_idx in self.RRTree])]
			result = []
			for idx in d:
				if len(result) < num_of_neighbor:
					heapq.heappush(result, idx)
				else:
					heap.heappushpop(result, idx)
			return [r[1] for r in result]

	def generate_new_node(self, start, rand):
		dx, dy = rand[0]-start[0], rand[1]-start[1]
		if dx == 0:
			return (x, y + self.Da, rand[2])
		theta = np.arctan(abs(dy/dx))
		x = start[0] + self.Da*np.cos(theta) if dx > 0 else start[0] - self.Da*np.cos(theta)
		y = start[1] + self.Da*np.sin(theta) if dy > 0 else start[1] - self.Da*np.sin(theta)
		return (x, y, rand[2])




