import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *

q_init = robot_poses["q_init"]
#q_init *= 0
sim = SoloSim(q_init)

print(sim.model.actuator_ctrlrange)

def simulate(steps=1000):
    for _ in range(steps):
        sim.robot.step()
        # if param == 'acc':
        #     print(sim.robot.data.qacc[0])
        # elif param == 'qpos':
        #     print(sim.robot.data.qpos[1])
        # elif param == 'ctrl':
        #     print(sim.robot.data.ctrl[0])
        sim.v.sync()
        sleep(0.01) # slow down the visualization

param = 'ctrl'
#simulate()
if param == 'acc':
    print(f'initial: {sim.robot.data.qacc[:sim.robot.ndof]}')
    qacc = np.array([1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
    sim.robot.set_qacc(qacc)
    print(f'initial 2: {sim.robot.data.qacc[:sim.robot.ndof]}')
    print(sim.robot.data.qacc[0])
    simulate()
    print(f'final: {sim.robot.data.qacc[:sim.robot.ndof]}')

elif param == 'qpos':
    print(f'initial: {sim.robot.data.qpos[:sim.robot.ndof]}')
    q = np.array([0, np.pi, 0, 0, np.pi, 0, 0, 0, 0, 0, 0, 0])
    sim.robot.set_q(q)
    simulate()
    print(f'final: {sim.robot.data.qpos[:sim.robot.ndof]}')

elif param == 'ctrl':
    print(f'initial: {sim.robot.data.ctrl}')
    q = q_init
    sim.robot.data.ctrl[-sim.robot.ndof:] = q
    simulate()
    print(f'final: {sim.robot.data.ctrl}')