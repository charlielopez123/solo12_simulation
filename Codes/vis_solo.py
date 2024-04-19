# Original Code inspired from visualize_solo.ipynb, first code to play around with Mujoco
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
np.random.seed(42)

EE_list = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']
#Initial rest pose
q_init = [0, np.pi/2,  np.pi, 
     0, np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi]

model = mujoco.MjModel.from_xml_path("scene.xml")
robot = Robot(model)

v = viewer.launch_passive(robot.model, robot.data)
#print(robot.get_q())

# Forward kinematics of current configuration
for EE in EE_list:
    x, R = robot.fk_pose(q = robot.get_q(), EE_name= EE)
    print(f"position of {EE}: {x}")

# Visualize a robot configuration
q = np.random.uniform(robot.qmin, robot.qmax)
print(f"random q: {q}")

#Initial rest pose
q = [0, np.pi/2,  np.pi, 
     0, np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi]

#Front arms up
q = [0, np.pi,  0, 
     0, np.pi,  0,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi]

# Place front feet on either side of the box
q = [np.pi/12, 3 * np.pi/2,  0, 
     - np.pi/12, 3 * np.pi/2,  0,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi]

# Setting q to robot
robot.set_q(q)
mujoco.mj_fwdPosition(robot.model, robot.data)
v.sync()

# Forward kinematics of current configuration
x, R = robot.fk_pose(robot.get_q())
print(x, R)

q_ik, success = robot.inverse_kinematics(np.array(x))
print(q_ik, success)



# animation 
for i in range(100):
    robot.set_q(q)

# base position 
x, _ = robot.fk_pose(q = robot.get_q(), EE_name= 'base_link')
print(f"position of base_link: {x}")

start_time = time.time()

t_max = 100
dt = 0.1
for t in np.arange(0, t_max, dt):
    robot.step()
    v.sync()

end_time = time.time() 
simulation_time = end_time - start_time
print(f"Simulation time: {simulation_time:.2f} seconds")