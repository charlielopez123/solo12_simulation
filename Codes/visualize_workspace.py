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

#q_min = np.zeros(12)
#q_max = 2*np.pi * np.ones(12)
max = 255
q_min = np.array([np.deg2rad(-75), -np.pi, -np.pi, np.deg2rad(-max), -np.pi, -np.pi, np.deg2rad(-75), -np.pi, -np.pi, np.deg2rad(-max), -np.pi, -np.pi])
q_max = np.array([np.deg2rad(max),  np.pi,  np.pi, np.deg2rad(75),    np.pi,  np.pi, np.deg2rad(max), np.pi, np.pi, np.deg2rad(75), np.pi, np.pi])

res =  20#resolution

q_range_1 = np.linspace(q_min[0], q_max[0], res) # (10, 3)
q_range_2 = np.linspace(q_min[1], q_max[1], res) # (10, 3)
q_range_3 = np.linspace(q_min[2], q_max[2], res) # (10, 3)
q_range_mesh = np.array(np.meshgrid(q_range_1, q_range_2, q_range_3)) # (3, 10, 10, 10)
# Reshape to (10, 10, 10, 3)
q_range_mesh = q_range_mesh.T # (10, 10, 10, 3)

x_FL = np.zeros((res, res, res, 3))
for i in range(res):
    for j in range(res):
        for k in range(res):
            x_FL[i, j, k] = sim.robot.fk_pose(q = q_range_mesh[i, j, k], EE_name = 'FL_FOOT')
            sim.visualize_point(x_FL[i, j, k], rgba=np.array([0, 0, 1, 0.8]))


q_comfortable = np.array([np.pi, np.pi/2, np.pi/2])
sim.animate(np.concatenate((q_comfortable, np.zeros(9))))

sim.visualize_point(key_points["box"]["left"], rgba=np.array([1, 0, 0, 1]))
sim.visualize_point(key_points["box"]["right"], rgba=np.array([1, 0, 0, 1]))
