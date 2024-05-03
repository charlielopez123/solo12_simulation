# Animation of the front EEs placing them up in the air to then place them on each side of the hypothetical box in front
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
q = robot_poses["q_arms_up_straight"]

sim.all_the_points()

sim.animate(q)
print("q_arms_up_straight")
sim.get_hand_positions()
#position of FL_FOOT: [0.18811347 0.14797154 0.35292119]
#position of FR_FOOT: [ 0.1881135  -0.14797147  0.35292121]

# Place front feet on either side of the box
q = robot_poses["q_hands_on_box_elbows_bent_under"]

sim.animate(q)
print("q_hands_on_box_elbows_bent_sideways")
sim.get_hand_positions()
#Hands on either side of box:
#position of FL_FOOT: [0.50831553 0.09773597 0.10973119]
#position of FR_FOOT: [ 0.50831553 -0.09773602  0.10973116]

#"q_hands_on_box_elbows_bent_sideways"s
#[0.51008737, 0.05419032, 0.09371348]
#[ 0.51008737, -0.05419032,  0.09371348]