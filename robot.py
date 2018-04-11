from euts import *
import baxter_jacobian as baxJ
from Jacobian import Jacobian

#%% Baxter Class
class Baxter:

	def __init__(self):
		self.lengths = {'l1': 0.27035, 'd1': 0.069, 'l2': 0.36435, 'd2': 0.069,
						'l3': 0.37429, 'd3': 0.01, 'l4': 0.229525, 'd4': 0}
		self.link_masses = {'m1':0.5, 'm2':0.5, 'm3':0.5, 'm4':0.5}
		
		self.angle_limits = {'s0': [-141, 51], 's1': [-123, 60], 
							'e0': [-173.5, 173.5], 'e1': [-3, 150], 
							'w0': [-175.25, 175.25], 'w1': [-90, 120],
							'w2': [-175.25, 175.25]}

		self.joint_keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
		self.task_pos_keys = ['x', 'y', 'z']
		self.task_rot_keys = ['rx', 'ry', 'rz']
		self.length_keys = ['l1', 'd1', 'l2', 'd2', 'l3', 'd3', 'l4', 'd4']
		self.DHref = self.computeDHref()

	def computeDHref(self):
		DHtable = {'s0':[self.lengths['l1'], 0, self.lengths['d1'], -90],
					's1':[0, 90, 0, 90],
					'e0':[self.lengths['l2'], 0, self.lengths['d2'], -90],
					'e1':[0, 0, 0, 90],
					'w0':[self.lengths['l3'], 0, self.lengths['d3'], -90],
					'w1':[0, 0, 0, 90],
					'w2':[self.lengths['l4'], 0, self.lengths['d4'], 0]}
		return DHtable

	def jacobian(self, joint_angles):
		j_matrix = baxJ.jacobian_matrix(joint_angles, self.lengths)
		J = Jacobian(j_matrix)
		return J

#TODO : Make generalized robot class for other robots