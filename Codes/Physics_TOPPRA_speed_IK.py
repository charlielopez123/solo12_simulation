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
sim = SoloSim(q_init = q_init)

sim.all_the_points()

sim.model.opt.timestep

sim.robot.data.ctrl = q_init

#sim.workspace(15)

points = key_movements['lift_arms']
timed = False

sim.simulate() #simulate forward in time to get started in the physics environment

# N_samples = len(points)
# duration, optimal_path = sim.TOPPRA(points)

sim.TOPPRA_speed_animate(points, ctrl = False)

# if timed:
#     start_time = time.time()

# q = sim.robot.get_q()
# for i in range(len(optimal_path)):
#     q = optimal_path[i]
#     sim.robot.set_q(q)
#     sim.robot.step()
#     if sim.v is not None:
#         sim.v.sync()

# if timed:
#     end_time = time.time()
#     simulation_time = end_time - start_time
