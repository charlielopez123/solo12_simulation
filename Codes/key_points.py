import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import robot_poses, EE_joints

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init)

sim.visualize_all_the_points()

# x_des for each EE
left_box_pos =[0.5, 0.06, 0.05]
right_box_pos = [ 0.5, -0.06,  0.05]

x_des = {'FL_FOOT': left_box_pos, 'FR_FOOT': right_box_pos, 'HL_FOOT': sim.robot.get_q()[EE_joints["HL_FOOT"]], 'HR_FOOT': sim.robot.get_q()[EE_joints["HR_FOOT"]]}

sim.visualize_point(left_box_pos)
sim.visualize_point(right_box_pos)

q, success = sim.inverse_kinematics_adjusted(x_des, q_ref = q_init, q = sim.robot.get_q())
print(q, success)

#sim.animate(q)