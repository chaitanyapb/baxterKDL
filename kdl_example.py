# Created on Fri Feb 16 20:13:27 2018
# Authors: Chaitanya Perugu

#%%
import time
import robot
import numpy as np
from FKSolver import FKSolver
from IKSolver import IKSolver

#%% Baxter Robot
baxter = robot.Baxter()

#%% Forward Pose Kinematics
baxfk = FKSolver(baxter)

print "Pose 1: End Pose"
joint_pose = {'s0':0, 's1':0, 'e0':0, 'e1':0, 'w0':0, 'w1':0, 'w2':0}
pose = baxfk.solveFK(joint_pose, visualize=False, output='pr')
print pose

#%% Velocity Kinematics: Jacobian
joint_pose = {'s0':-30, 's1':-10, 'e0':-10, 'e1':30, 'w0':-15, 'w1':30, 'w2':10}
J = baxter.jacobian(joint_pose)
print "Jacobian = "
J.display()

print "Is Jacobian singular? ", J.is_singular()
print "Pseudoinverse = "
print np.around(J.pinv(), 3)
print "Null projector = "
print np.around(J.nullprojector(), 3)
print "Yoshikawa Manipulability Index = "
print J.yoshikawa()

#%% Inverse Pose Kinematics
baxik = IKSolver(baxter)

end_tpose = {'x':0.1, 'y':0, 'z':0.25, 'rx':0, 'ry':0, 'rz':0}

start_time = time.time()
ik_ans = baxik.PsuedoInverse(end_tpose)
print "Time taken = ", time.time() - start_time, "s"

print "Found IK Solution = "
print ik_ans