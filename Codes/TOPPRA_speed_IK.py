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

#get the arms up to the strating position
x_des = sim.x_des(target = key_points["arms_up"])
q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref)
sim.speed_animate(q, num_time_steps= 3, t_max = 3)

#parameters
points = [
    key_points["arms_up"],
    key_points["via_point1_get2box"],
    key_points["via_point2_get2box"],
    key_points["box_high"]
    ]
timed = True

N_samples = len(points)
num_time_steps = 100
t_max, optimal_velocities = sim.TOPPRA(N_samples, points)
print(f'duration: {t_max}')
#print(f'optimal_velocities: {optimal_velocities}')


dt = t_max/num_time_steps
print(f'dt: {dt}')
q_1 = sim.robot.get_q()
if timed:
    start_time = time.time()
q = q_1
for i in range(num_time_steps):
    velocities = optimal_velocities[i]
    q += velocities*dt
    sim.animate(q_2=q, t_max = dt, dt = dt/2)
if timed:
    end_time = time.time()
    simulation_time = end_time - start_time
    print(f'time: {simulation_time}')