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

#Front arms up
q = robot_poses["q_arms_up"]

sim.animate(q)
print("q_init")
sim.get_hand_positions()
#position of FL_FOOT: [0.18811347 0.14797154 0.35292119]
#position of FR_FOOT: [ 0.1881135  -0.14797147  0.35292121]

# Place front feet on either side of the box
q = robot_poses["q_hands_on_box"]

sim.animate(q)
print("q_hands_on_box")
sim.get_hand_positions()
#Hands on either side of box:
#position of FL_FOOT: [0.50831553 0.09773597 0.10973119]
#position of FR_FOOT: [ 0.50831553 -0.09773602  0.10973116]
