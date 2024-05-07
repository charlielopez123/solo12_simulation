# Script to test out function speed_animate from Simulation.py, animating the movements of the robot with a velocity profile

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

sim.speed_animate(q, num_time_steps= 2, t_max = 2,timed=True) # works: num_time_steps= 2, t_max = 2 
