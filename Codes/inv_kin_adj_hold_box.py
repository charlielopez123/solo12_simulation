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

sim.all_the_points()


#sim.animate(robot_poses["q_arms_up_straight"])
#sim.get_hand_positions()

target = key_points["arms_up"]
x_des ={'FL_FOOT': target["left"],
        'FR_FOOT': target["right"],
        'HL_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HL_FOOT"), #keep current EE position for Hind Legs
        'HR_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HR_FOOT")} #keep current EE position for Hind Legs
q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref)
sim.animate(q)

target = key_points["box_high"]
x_des ={'FL_FOOT': target["left"],
        'FR_FOOT': target["right"],
        'HL_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HL_FOOT"), #keep current EE position for Hind Legs
        'HR_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HR_FOOT")} #keep current EE position for Hind Legs
q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref)
sim.animate(q)

#sim.animate(q_ref)

target = key_points["box_up"]
x_des ={'FL_FOOT': target["left"],
        'FR_FOOT': target["right"],
        'HL_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HL_FOOT"), #keep current EE position for Hind Legs
        'HR_FOOT': sim.robot.fk_pose(q=sim.robot.get_q(), EE_name="HR_FOOT")} #keep current EE position for Hind Legs
q = sim.inverse_kinematics_adjusted(x_des, q_ref = robot_poses["q_hold_box_up"])
sim.animate(q)




#<key qpos='0.2447 4.12114 -0.60945 -0.2447 4.12114 -0.60945 2.94472e-05 -1.57079 3.14159 -2.94472e-05 -1.57079 3.14159'/>#