"""
This script simulates and visualizes the motion of a robot picking up a box, following a trajectory optimization policy.
Using the TOPPRA_speed_animate() function in Simulation.py to animate the movement using the generated velocity profiles from TOPPRA() in Simulation.py

Note: This script must be launched with an mjpython kernel to visualize the animation.

Steps:
1. Initialize the simulation with initial joint positions.
2. Iterate through key movements to calculate optimal trajectories using the TOPP-RA algorithm.
"""
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *


q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init, use_hind_legs=False, use_box=True)
#redefined the velocity and acceleration limits to pick up the box
sim.vlim = 10 
sim.alim = 15

#Show all the points
sim.all_the_points()

#Whether to simulate the physics or not
ctrl = True # ctrl is True when simulating with physics, false if not

#Initialize duration
duration = 0
# Iterate through key movements to calculate and animate optimal trajectories
if not sim.use_hind_legs:
    for mvmt in key_movements:
        points = key_movements[mvmt]
        dur = sim.TOPPRA_speed_animate(points, ctrl = ctrl, plot = True)
        duration += dur[0]
else:
    for mvmt in key_movements_hind_legs:
        points = key_movements_hind_legs[mvmt]
        dur = sim.TOPPRA_speed_animate(points, ctrl = ctrl, plot = True)
        duration += dur[0]

print(duration)