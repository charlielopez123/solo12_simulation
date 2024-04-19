# Testing the inverse_kinematics() funtion from solo.py with q_init initialization
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import robot_poses, EE_joints


#Initial rest pose
q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init)

#print(sim.robot.get_q())

# FL_FOOT Position when arms in the air
FL_FOOT_pos = [0.15061844, 0.06616774, 0.35240609]
FR_FOOT_pos = [ 0.15061846, -0.06616772,  0.35240609]
#FL_FOOT_pos = [0.5, 0.06, 0.05]

sim.visualize_point(FL_FOOT_pos)
sim.visualize_point(FR_FOOT_pos)

q = sim.robot.get_q()
EE = 'FL_FOOT'
q_ik, success = sim.robot.inverse_kinematics(FL_FOOT_pos, q_ref = q_init, EE_name=EE)
print(f"for {EE}: {success}")
for i in EE_joints[EE]: # change the relevant joints of the EE
        q[i] = q_ik[i]
EE = 'FR_FOOT'
q_ik, success = sim.robot.inverse_kinematics(FR_FOOT_pos, q_ref = q_init, EE_name=EE)
print(f"for {EE}: {success}")
for i in EE_joints[EE]: # change the relevant joints of the EE
        q[i] = q_ik[i]

#print(q, success)

#test obtained q
sim.animate(np.array(q), timed = True)
sim.get_hand_positions()