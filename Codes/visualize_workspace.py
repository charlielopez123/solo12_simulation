# Testing of inverse_kinematics_adjusted() function for holding the box up position using 'q_arms_up' as a q_ref for the solver
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init)
# robot = Robot(mujoco.MjModel.from_xml_path('scene.xml'), q_init = q_init)

q_min = np.zeros(12)
q_max = 2*np.pi * np.ones(12)

res = 20 #resolution

q_range = np.linspace(0, 2*np.pi, res) # (10, 3)
q_range_mesh = np.array(np.meshgrid(q_range, q_range, q_range)) # (3, 10, 10, 10)
# Reshape to (10, 10, 10, 3)
q_range_mesh = q_range_mesh.T # (10, 10, 10, 3)
print(q_range_mesh)

x_FL = np.zeros((res, res, res, 3))
for i in range(res):
    for j in range(res):
        for k in range(res):
            x_FL[i, j, k] = sim.robot.fk_pose(q = q_range_mesh[i, j, k], EE_name = 'FL_FOOT')
            sim.visualize_point(x_FL[i, j, k], rgba=np.array([0, 0, 1, 0.8]))

print(q_range_mesh[0, 0, 0])
print(x_FL[0, 0, 0])

q_comfortable = np.array([np.pi, np.pi/2, np.pi/2])
sim.animate(np.concatenate((q_comfortable, np.zeros(9))))

sim.visualize_point(key_points["box"]["left"], rgba=np.array([1, 0, 0, 1]))
sim.visualize_point(key_points["box"]["right"], rgba=np.array([1, 0, 0, 1]))
