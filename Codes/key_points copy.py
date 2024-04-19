import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import robot_poses, EE_joints, key_points

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init)

sim.visualize_all_the_points()

x_des ={'FL_FOOT': key_points["box_up"]["left"],
        'FR_FOOT': key_points["box_up"]["left"],
        'HL_FOOT': sim.robot.get_q()[EE_joints["HL_FOOT"]],
        'HR_FOOT': sim.robot.get_q()[EE_joints["HR_FOOT"]]}

q, success = sim.inverse_kinematics_adjusted(x_des, q_ref = q_init, q = sim.robot.get_q())
#print(q, success)

#sim.animate(q)