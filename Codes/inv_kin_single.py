# Testing the inverse_kinematics() on a single EE funtion from solo.py with q_init initialization a
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *


#Initial rest pose
q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init)

#print(sim.robot.get_q())

# FL_FOOT Position when arms in the air
FL_FOOT_pos = [0.15061844, 0.06616774, 0.35240609]
FR_FOOT_pos = [ 0.15061846, -0.06616772,  0.35240609]
#FL_FOOT_pos = [0.5, 0.06, 0.05]

sim.visualize_point(FL_FOOT_pos)
#sim.visualize_point(FR_FOOT_pos)

EE = 'FL_FOOT'
sim.get_joint_positions(EE)
# Determine a q_ref that is close to the initial solution for with all the other EE at their current position
q_ref = sim.robot.get_q()
for joint in EE_joints[EE]:
    q_ref[joint] = robot_poses["q_arms_up"][joint]
q_ik, success = sim.robot.inverse_kinematics(FL_FOOT_pos, q_ref = q_ref, EE_name=EE)
print(f"for {EE}: {success}")

#print(q success)

#test obtained q
sim.animate(np.array(q_ik), timed = True)
sim.get_joint_positions(EE)