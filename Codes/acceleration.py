import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *



q_init = robot_poses["q_init"]
sim = SoloSim()

def simulate(steps=1000):
        for _ in range(steps):
            sim.robot.step()
            # Optionally, update any external observers or render the state
            sim.v.sync()

print(sim.robot.data.qacc)
qacc = np.array([1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
print(len(qacc))
sim.robot.set_qacc(qacc)
simulate()
print(sim.robot.data.qacc)