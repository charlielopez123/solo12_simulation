#Visualize a certain configuration in the Mujoco viewe
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

sleep(1)
q = q_ref # Select a configuration from Joint_positions.py
sim.robot.set_q(q, ctrl = False)
sim.robot.forward()
sim.v.sync()
