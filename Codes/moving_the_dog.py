import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time

model = mujoco.MjModel.from_xml_path("scene.xml")
robot = Robot(model)

v = viewer.launch_passive(robot.model, robot.data)
v.sync()

#Initial rest pose
q = [0, np.pi/2,  np.pi, 
     0, np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi]