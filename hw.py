import pdb
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math

default_map = []

class robot:
	def __init__(self, initial_idx, goal_idx, Da=8, e_space=default_map):
		self.initial_idx = initial_idx
		self.goal_idx = goal_idx
		self.Da = Da
		self.o_space = o_space
		self.max_fail = 10**4
		self.found_path = False
		# each node: 'idx':(x, y, theta), 'parent' = #, 'distance' = d
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

	def nearest_neighbor(self, num_of_neighbor, target_idx):
		if num_of_neighbor == 1:
			

