"""
This script simulates the motion of a robot by generating a trajectory that follows a jerk minimization policy.
It outputs plots of optimal joint positions, velocities, and accelerations over time.

Note: The animation is not visualized here because the MuJoCo viewer can only be used with an mjpython kernel.

Steps:
1. Initialize the simulation with initial joint positions.
2. Iterate through key points to calculate desired end-effector positions and solve inverse kinematics.
3. Compute optimal trajectories and animate them.
4. Store the computed positions, velocities, and accelerations.
5. Create time steps for the entire duration of the simulation.
6. Plot joint positions, velocities, and accelerations over time.

To run the script:
1. Ensure you have the necessary dependencies installed.
2. Run the script like a normal Python script.
3. The script will display plots of optimal joint positions, velocities, and accelerations over time.
"""
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *
import matplotlib.pyplot as plt

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init, launch_viewer=False)

pos = []
vel = []
acc = []

for idx, i in enumerate(["initial_pos", "arms_up", "via_point1_get2box","box_high", "via_point1_lift_box", "box_on_back_halfway","box_up", "box_on_back"]):
    x_des = sim.x_des(target = key_points[i])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = key_points[i]['q_ref'])
    optimal_positions, optimal_velocities, optimal_accelerations = sim.velocity_profile(dt = 0.2, full_duration = 2, current = sim.robot.get_q(), desired=q)
    sim.animate(np.transpose(optimal_positions)[-1])
    if idx == 0:
        pos = np.copy(optimal_positions)
        vel = np.copy(optimal_velocities)
        acc = np.copy(optimal_accelerations)
    else:
        pos= np.concatenate((pos, optimal_positions), axis = -1)
        vel= np.concatenate((vel, optimal_velocities), axis = -1)
        acc= np.concatenate((acc, optimal_accelerations), axis = -1)

print(np.shape(pos), np.shape(vel), np.shape(acc))
time_steps = np.linspace(0, 15, num=np.shape(pos)[1])
print(np.shape(time_steps))

plt.figure(figsize=(12, 9))

for i in range(12):
    optimal_positions = pos[i]
    optimal_velocities = vel[i]
    optimal_accelerations = acc[i]
    
    # Plot positions
    plt.subplot(3, 1, 1)
    plt.plot(time_steps, optimal_positions, label=f'{name_joints[i]}')
    
    # Plot velocities
    plt.subplot(3, 1, 2)
    plt.plot(time_steps, optimal_velocities, label=f'{name_joints[i]}')
    
    # Plot accelerations
    plt.subplot(3, 1, 3)
    plt.plot(time_steps, optimal_accelerations, label=f'{name_joints[i]}')

# Formatting the plots
plt.subplot(3, 1, 1)
plt.title('Optimal Joint Positions')
plt.xlabel('Time [s]')
plt.ylabel('Position [rad]')
#plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.title('Optimal Joint Velocities')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
#plt.legend()
plt.grid(True)

plt.subplot(3, 1, 3)
plt.title('Optimal Joint Accelerations')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [rad/s²]')
#plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
