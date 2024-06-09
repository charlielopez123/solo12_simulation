"""
This script is a test of simulating the motion of a robot by generating a trajectory that follows a jerk minimization policy.
It outputs plots of optimal joint positions, velocities, and accelerations over time.
This code is similar to 'plot_jerk_min_trajectory' with the additional use of viapoints

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
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from Joint_positions import *

# Assuming `robot_poses` and `name_joints` are already defined
num_time_steps = 10
full_duration = 3  # full duration of the movement
dt = full_duration / num_time_steps

current = robot_poses['q_init']
desired = robot_poses['q_arms_up_straight']

# Define the via points
via_points = [
    {'time_step': 4, 'positions': np.array([0.2, -0.1, 0.3, 0.5, -0.2, 0.1, 0.2, -0.3, 0.4, 0.1, -0.2, 0.2])},
    {'time_step': 7, 'positions': np.array([0.3, -0.2, 0.4, 0.6, -0.3, 0.2, 0.3, -0.4, 0.5, 0.2, -0.3, 0.3])}
]

# Define the objective function to minimize jerk
def objective_function(vars):
    smoothness_measure = 0
    for i in range(12):
        # Extract the velocities for the i-th joint
        velocities = vars[12 * num_time_steps + i * num_time_steps: 12 * num_time_steps + (i + 1) * num_time_steps]
        accelerations = np.diff(velocities, prepend=0) / dt
        jerk = np.diff(accelerations, prepend=0) / dt
        smoothness_measure += np.sum(jerk**2)
    return smoothness_measure

# Define the constraints
constraints = []

# Position constraints at start and end
for i in range(12):
    # Start position constraint
    def start_pos_constraint(vars, i=i):
        return vars[i * num_time_steps] - current[i]
    constraints.append({'type': 'eq', 'fun': start_pos_constraint})
    
    # End position constraint
    def end_pos_constraint(vars, i=i):
        return vars[i * num_time_steps + (num_time_steps - 1)] - desired[i]
    constraints.append({'type': 'eq', 'fun': end_pos_constraint})

# Velocity constraints at start and end (assuming they should be zero)
for i in range(12):
    # Start velocity constraint
    def start_vel_constraint(vars, i=i):
        return vars[12 * num_time_steps + i * num_time_steps]
    constraints.append({'type': 'eq', 'fun': start_vel_constraint})
    
    # End velocity constraint
    def end_vel_constraint(vars, i=i):
        return vars[12 * num_time_steps + i * num_time_steps + (num_time_steps - 1)]
    constraints.append({'type': 'eq', 'fun': end_vel_constraint})

# Positional evolution constraint (position at t+1 = position at t + velocity * dt)
for i in range(12):
    for t in range(num_time_steps - 1):
        def pos_evolution_constraint(vars, i=i, t=t):
            return vars[i * num_time_steps + t + 1] - (vars[i * num_time_steps + t] + vars[12 * num_time_steps + i * num_time_steps + t] * dt)
        constraints.append({'type': 'eq', 'fun': pos_evolution_constraint})

# Constraints for via points
for via_point in via_points:
    time_step = via_point['time_step']
    positions = via_point['positions']
    for i in range(12):
        def via_point_constraint(vars, i=i, time_step=time_step, positions=positions):
            return vars[i * num_time_steps + time_step] - positions[i]
        constraints.append({'type': 'eq', 'fun': via_point_constraint})

# Initial guess: interpolate positions, zero velocities
initial_positions = np.linspace(current, desired, num=num_time_steps).flatten()
initial_velocities = np.zeros(12 * num_time_steps)
initial_guess = np.concatenate([initial_positions, initial_velocities])

# Check the objective function value at the initial guess
print(f'Initial objective function value: {objective_function(initial_guess)}')

# Run the optimization
result = minimize(fun=objective_function, x0=initial_guess, method='SLSQP', constraints=constraints)

# Extract and print the optimal positions and velocities
if result.success:
    for i in range(12):
        optimal_positions = result.x[i * num_time_steps: (i + 1) * num_time_steps]
        optimal_velocities = result.x[12 * num_time_steps + i * num_time_steps: 12 * num_time_steps + (i + 1) * num_time_steps]
        print(f'Optimal positions of {name_joints[i]}: {optimal_positions}')
        print(f'Optimal velocities of {name_joints[i]}: {optimal_velocities}')
else:
    print("Optimization failed:", result.message)

# Plot the position, velocity, and acceleration profiles
if result.success:
    time_steps = np.linspace(0, full_duration, num=num_time_steps)
    
    plt.figure(figsize=(12, 9))
    
    for i in range(12):
        optimal_positions = result.x[i * num_time_steps: (i + 1) * num_time_steps]
        optimal_velocities = result.x[12 * num_time_steps + i * num_time_steps: 12 * num_time_steps + (i + 1) * num_time_steps]
        optimal_accelerations = np.diff(optimal_velocities, prepend=0) / dt
        
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
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.title('Optimal Joint Velocities')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.title('Optimal Joint Accelerations')
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [rad/s²]')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
