# Created on Tue Apr 10 21:21:50 2018
# Author: Chaitanya Pb

from FKSolver import FKSolver
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#%% Workspace
def workspace(robot, num_samples):
	robotfk = FKSolver(robot)

	s0_limits = robot.angle_limits['s0']
	s1_limits = robot.angle_limits['s1']
	e0_limits = robot.angle_limits['e0']
	e1_limits = robot.angle_limits['e1']
	w0_limits = robot.angle_limits['w0']
	w1_limits = robot.angle_limits['w1']
	w2_limits = robot.angle_limits['w2']

	s0_range = np.linspace(s0_limits[0], s0_limits[1], num_samples)
	s1_range = np.linspace(s1_limits[0], s1_limits[1], num_samples)
	e0_range = np.linspace(e0_limits[0], e0_limits[1], num_samples)
	e1_range = np.linspace(e1_limits[0], e1_limits[1], num_samples)
	w0_range = np.linspace(w0_limits[0], w0_limits[1], num_samples)
	w1_range = np.linspace(w1_limits[0], w1_limits[1], num_samples)
	w2_range = np.linspace(w2_limits[0], w2_limits[1], num_samples)

	ws_x = []; ws_y = []; ws_z = []
	posesAnalysed = 0

	for s0 in s0_range:
		for s1 in s1_range:
			for e0 in e0_range:
				for e1 in e1_range:
					for w0 in w0_range:
						for w1 in w1_range:
							joint_pose = {'s0':s0, 's1':s1, 'e0':e0, 'e1':e1, 
											'w0':w0, 'w1':w1, 'w2':0}
							T = robotfk.solveFK(joint_pose)
							ws_x += [T.item((0, 3))]
							ws_y += [T.item((1, 3))]
							ws_z += [T.item((2, 3))]
							posesAnalysed += 1

	print "Poses Analysed = {}".format(posesAnalysed)

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(ws_x, ws_y, ws_z)
	ax.scatter(0, 0, 0, c='r', marker='o')
	ax.set_xlabel('X (m)')
	ax.set_ylabel('Y (m)')
	ax.set_zlabel('Z (m)')
	plt.show()