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

sim.visualize_all_the_points()

x_des ={'FL_FOOT': key_points["box_up"]["left"],
        'FR_FOOT': key_points["box_up"]["right"],
        'HL_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HL_FOOT"), #keep current EE position for Hind Legs
        'HR_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HR_FOOT")} #keep current EE position for Hind Legs

q = sim.inverse_kinematics_adjusted(x_des, q_ref = robot_poses["q_arms_up"])
sim.animate(q)