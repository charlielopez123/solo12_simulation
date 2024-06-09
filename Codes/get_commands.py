"""
This script demonstrates the simulation of a full task using TOPPRA optimized trajectories and writes the obtained commands to a `commands.json` file for execution on the masterboard of the Solo12 robot.
This script defines the commands for the full box grasping using the hind_legs due to isseus with the front legs of the Solo12. Can simply switch use_hind_legs = false to switch
1. Initialization and Setup:
   - Modules such as Mujoco, Solo robot class, simulation, and joint positions are initialized.
   - Initial joint configuration of the robot is set.

2. Simulation and Optimization:
   - SoloSim class is instantiated with the initial configuration, set to use hind legs.
   - Workspace visualization for hind legs (optional).
   - Iterates through key movements for hind legs.
   - Calculates optimal velocity profiles using TOPPRA.
   - Animates robot's motion in the simulation using the optimized trajectories.

3. Data Collection:
   - Positions, velocities, and accelerations from optimized trajectories are stored.

4. Conversion and Storage:
   - Accumulates total time for all movements.
   - Converts time, positions, velocities, and accelerations arrays to lists.

5. Writing Commands to JSON File:
   - Writes obtained commands to a `commands.json` file.
   - JSON likely contains commands for Solo12 masterboard (e.g., joint positions, velocities) for executing the simulated task on the physical robot.
"""
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *
import matplotlib.pyplot as plt
import json

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init, use_hind_legs=True)

if sim.use_hind_legs:
    sim.all_the_points(key_points_hind_legs)

see_workspace = False
if see_workspace:
    sim.workspace(res = 20, EE_name='HL_FOOT')

positions_sim= []
positions_TOPPRA= []
velocities= []
accelerations= []

full_time = 0
if sim.use_hind_legs:
    for mvmt in key_movements_hind_legs:
        points = key_movements_hind_legs[mvmt]
        duration, pos_sim, pos_TOPPRA, vel, acc = sim.TOPPRA_speed_animate(points, ctrl = True, plot = True)
        print(pos_sim.shape, pos_TOPPRA.shape, vel.shape, acc.shape)
        positions_sim.append(pos_sim)
        positions_TOPPRA.append(pos_TOPPRA)
        velocities.append(vel)
        accelerations.append(acc)
        full_time += duration
else:
    for mvmt in key_movements:
        points = key_movements[mvmt]
        duration, pos_sim, pos_TOPPRA, vel, acc = sim.TOPPRA_speed_animate(points, ctrl = True, plot = True)
        print(pos_sim.shape, pos_TOPPRA.shape, vel.shape, acc.shape)
        positions_sim.append(pos_sim)
        positions_TOPPRA.append(pos_TOPPRA)
        velocities.append(vel)
        accelerations.append(acc)
        full_time += duration

positions_sim = np.concatenate(positions_sim, axis=0)
positions_TOPPRA = np.concatenate(positions_TOPPRA, axis=0)
velocities = np.concatenate(velocities, axis=0)
accelerations = np.concatenate(accelerations, axis=0)
print(positions_sim.shape)
print(f'full_time: {full_time}')

ts_sample = np.linspace(0, full_time, len(positions_sim))
print(ts_sample.shape, positions_sim.shape, velocities.shape, accelerations.shape)

# Convert arrays to lists
array1_list = ts_sample.tolist()
array2_list = positions_sim.tolist()
array3_list = velocities.tolist()
array4_list = accelerations.tolist()

data = {
    "time_stamps": array1_list,
    "joint_positions": array2_list,
    "joint_velocities": array3_list,
    "joint_accelerations": array4_list
}

# Write the dictionary to a JSON file
with open('commands.json', 'w') as f:
    json.dump(data, f)
