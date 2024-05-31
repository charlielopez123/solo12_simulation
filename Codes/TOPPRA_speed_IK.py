# Testing of inverse_kinematics_adjusted() function for holding the box up position using 'q_arms_up' as a q_ref for the solver
#Using the TOPPRA_speed_animate() function in Simulation.py to animate the movement using the generated velocity profiles from TOPPRA() in Simulation.py
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init, use_hind_legs=False)
sim.vlim = 3
sim.alim = 5

sim.all_the_points()

duration = 0
if not sim.use_hind_legs:
    for mvmt in key_movements:
        points = key_movements[mvmt]
        dur = sim.TOPPRA_speed_animate(points, ctrl = True, plot = True)
        duration += dur[0]
else:
    for mvmt in key_movements_hind_legs:
        points = key_movements_hind_legs[mvmt]
        dur = sim.TOPPRA_speed_animate(points, ctrl = True, plot = True)
        duration += dur[0]

print(duration)
