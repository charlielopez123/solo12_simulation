# remember to set "base_link" free in solo12.xml
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time

model = mujoco.MjModel.from_xml_path("scene.xml")
robot = Robot(model)

def simulate(robot, t_max, dt, timed = False):
     if timed:
            start_time = time.time()
     for t in np.arange(0, t_max, dt):
        robot.step()
        v.sync()
     if timed: 
          end_time = time.time() 
          simulation_time = end_time - start_time
          print(f"Simulation time: {simulation_time:.2f} seconds")

def animate(robot, q_1, q_2, t_max, dt, timed = False):
     if timed:
               start_time = time.time()
     for t in np.arange(0, t_max, dt):
          robot.set_q(t/t_max * (q_2 - q_1) + q_1)
          print(robot.get_q()[2])
          robot.step()
          v.sync()
          sleep(dt)
     if timed: 
          end_time = time.time() 
          simulation_time = end_time - start_time
          print(f"Simulation time: {simulation_time:.2f} seconds")

v = viewer.launch_passive(robot.model, robot.data)
simulate(robot, t_max = 3, dt= 0.01, timed = True)

# raise left forearm
q_init = np.zeros_like(robot.get_q())
# raise forearm with FL_KFE
q_init[2] = np.pi/2

animate(robot, q_1 = robot.get_q(), q_2 = q_init , t_max = 3, dt= 0.01, timed = False)


# reset all q to zeros as free base link falsifies initial q values
robot.get_q()
#Out[2]: array([0., 0., 1., 1., 0., 0., 0., 0., 0., 0., 0., 0.])
robot.set_q(np.zeros_like(robot.get_q()))
robot.get_q()
# Out[4]: array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])