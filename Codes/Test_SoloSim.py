import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import robot_poses

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init)

sim.visualize_point(key_pos=[0.18811347, 0.14797154, 0.35292119])
sim.visualize_point(key_pos=[0.1881135, -0.14797147, 0.35292121])
q_arms_up = robot_poses["q_arms_up"]
sim.animate(q_2 = q_arms_up, timed = True)
#position of FL_FOOT: [0.18811347 0.14797154 0.35292119]
#position of FR_FOOT: [ 0.1881135  -0.14797147  0.35292121]

q_hands_on_box = robot_poses["q_hands_on_box"]
sim.animate(q_2 = q_hands_on_box, timed = True)
#Hands on either side of box:
#position of FL_FOOT: [0.50831553 0.09773597 0.10973119]
#position of FR_FOOT: [ 0.50831553 -0.09773602  0.10973116]

q_hold_box_up = robot_poses["q_hold_box_up"]
sim.animate(q_2 = q_hold_box_up, timed = True)


q = sim.robot.q_home.copy()
q[1] = 0
q[0] = np.deg2rad(-75)
sim.animate(q_2 = q, timed = True)