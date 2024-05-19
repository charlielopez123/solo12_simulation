import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Joint_positions import *
from scipy.optimize import minimize

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt

class SoloSim:
  """
  A class for simulating a Solo robot in a MuJoCo environment.
  """

  def __init__(self,
            q_init=None,
            launch_viewer = True):
    """
    Initializes the simulation environment.

    Args:
      xml_path: Path to the MuJoCo scene XML file.
      q_init: Initial joint positions for the robot (optional).
    """
    # Load scene and model
    self.model = mujoco.MjModel.from_xml_path('scene.xml')
    self.robot = Robot(self.model, q_init)

    # Launch viewer
    if launch_viewer:
      self.v = viewer.launch_passive(self.robot.model, self.robot.data)

    #Mujoco time step when using mj_step()
    self.dt_control = self.model.opt.timestep

    #If in a context of a manipulation task, skip the IK computation of the hind legs
    self.manipulation_task = True

  def animate(self, q_2, q_1 = None, t_max = 2, dt = 0.01, timed=False):
      """
      Animates a transition between two robot joint configurations.

      Args:
        q_1: Starting joint positions.
        q_2: Ending joint positions.
        t_max: Maximum simulation time.
        dt: Time step for simulation.
        timed: Whether to print the simulation time (optional).
      """
      if q_1 is None:
        q_1 = self.robot.get_q()
      if timed:
        start_time = time.time()
      for t in np.arange(0, t_max, dt):
        self.robot.set_q(t / t_max * (q_2 - q_1) + q_1, ctrl=False)
        #self.robot.step()
        self.robot.forward()
        if self.v is not None:
          self.v.sync()
        sleep(dt)
      if timed:
        end_time = time.time()
        simulation_time = end_time - start_time
        print(f"Simulation time: {simulation_time:.2f} seconds")

  def get_hand_positions(self):
      """
      Retrieves the positions of the robot's front "hands".

      Returns:
        A dictionary containing the positions of the FL_FOOT and FR_FOOT.
      """
      positions = {}
      # FL_FOOT Position
      x = self.robot.fk_pose(q=self.robot.get_q(), EE_name="FL_FOOT")
      positions["FL_FOOT"] = x
      # FR_FOOT Position
      x = self.robot.fk_pose(q=self.robot.get_q(), EE_name="FR_FOOT")
      positions["FR_FOOT"] = x
      return positions

  def visualize_point(self, key_pos, rgba=np.array([0.8, 0.3, 0.3, 0.5]), verbose=False):
      """
      Visualizes a point in the MuJoCo viewer.

      Args:
        key_pos: The position of the point to visualize.
        id: Optional ID for the geometry (used for potential reuse).
      """
      sphere_radius = 0.01

      if self.v is None:
        print("Viewer not initialized, cannot visualize point.")
        return
      
      # increment number of geometries
      self.v.user_scn.ngeom += 1
      id = self.v.user_scn.ngeom - 1

      # Create a new sphere geometry
      geom_id = mujoco.mjv_initGeom(
        self.v.user_scn.geoms[id],  # You can specify an existing geometry ID for reuse
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[sphere_radius, 0, 0],
        pos=key_pos,
        mat=np.eye(3).flatten(),  # Material properties (identity for now)
        rgba=rgba,  # Reddish color with transparency
      )
      #v.user_scn.ngeom = id
      self.v.sync()

      # Handle potential errors (optional)
      if geom_id == -1:
        print("Error creating geometry")
      else:
        if verbose:
          print(f"Point visualized with geometry ID")
          print({self.v.user_scn.geoms[id]})
        self.v.sync()

  def inverse_kinematics_adjusted(self, x_des, q_ref=None, q = None):
    """
    Inverse kinematics function calling the inverse_kinematics function from solo.py for a desired position of an end-effector
    which is then adjusted by keeping all the other joints unaffecting the chosen EE's position to the current configuration

    Args:
      x_des: desired position of the end-effector [x, y, z]
      q_ref: reference joint position from which the inverse kinematics are computed from, by default set as None
      EE_name: End-effector name from ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"], by default "FR_FOOT",
      q: current joint disposition before setting it to the robot

    Returns:
    q: solution returned by the solver for the joint positions to reach the EE to x_des position
    success: True or False, whether the solver managed to converge towards a solution or not
    """

    if q_ref is None:
      q_ref = self.robot.get_q()

    if q is None:
      q = self.robot.get_q()

    noise = self.build_noise()

    if self.manipulation_task == True:
      for EE in ['FL_FOOT', 'FR_FOOT']:
        q_ik, success = self.robot.inverse_kinematics(x_des[EE], q_ref = q_ref, EE_name = EE, noise = noise)
        print(f"for {EE}: {success}")
        for i in EE_joints[EE]: # change the relevant joints of the EE
          q[i] = q_ik[i]
    else:
      for EE in EE_joints:
        q_ik, success = self.robot.inverse_kinematics(x_des[EE], q_ref = q_ref, EE_name = EE, noise = noise)
        print(f"for {EE}: {success}")
        for i in EE_joints[EE]: # change the relevant joints of the EE
          q[i] = q_ik[i]

    return q

  def all_the_points(self):
    """
    Visualize all the points in the viewer
    """
    for situation in key_points:
      rgba = key_points[situation]['rgba']
      for EE in key_points[situation]['pos']:
        self.visualize_point(key_points[situation]['pos'][EE], rgba)

  def get_joint_positions(self, EE):
    """
    Prints the positions of the chosen EE's corresponding joints with their names

    Args:
      EE : Chosen EE from which we want information on the corresponding joints
    """

    q = self.robot.get_q()
    for joint in EE_joints[EE]:
      print(f"Joint {name_joints[joint]} Position: {q[joint]}")

  def build_noise(self):
    # Build the noise array that we add to the x0 initial solution of th IK Solver
    # We want a symmetric noise as to converge to a symmetric solution for both EE

    # Generate three random normal numbers for additional noise given to an EE
    random_numbers = np.random.normal(size=3) # [a, b,c]
    random_numbers_negative = random_numbers * np.array([-1, 1, 1]) # [-a, b, c]

    pattern = np.concatenate((random_numbers, random_numbers_negative)) # [a, b, c, -a, b, c]
    #Build the noise array to add to each of the initial solution of the solver
    noise = np.tile(pattern, 2)*0.001 # [a, b, c, -a, b, c, a, b, c, -a, b, c]

    return noise

  def workspace(self, res = 10, rgba = np.array([0, 0, 1, 0.8]), EE_name = 'FL_FOOT'):
    """
    Visualizes a cloud of possible EE positions as to get a rough estimate of the EE's workspace

    Args:
      res: resolution of joint positional increments
      rgba: rgb color and opacity of workspace points
      EE_name : Chosen EE from which we want information on the corresponding workspace
    """
    # q_min = np.zeros(12)
    # q_max = 2*np.pi * np.ones(12)
    q_min = np.array([np.deg2rad(-75), -np.pi, -np.pi, np.deg2rad(-255), -np.pi, -np.pi, np.deg2rad(-75), -np.pi, -np.pi, np.deg2rad(-255), -np.pi, -np.pi])
    q_max = np.array([np.deg2rad(255),  np.pi,  np.pi, np.deg2rad(75),    np.pi,  np.pi, np.deg2rad(255), np.pi, np.pi, np.deg2rad(75), np.pi, np.pi])

    q_range = np.linspace(0, 2*np.pi, res) # (10, 3)
    q_range_mesh = np.array(np.meshgrid(q_range, q_range, q_range)) # (3, 10, 10, 10)
    # Reshape to (10, 10, 10, 3)
    q_range_mesh = q_range_mesh.T # (10, 10, 10, 3)

    index = EEs.index(EE_name) # Which index of EE is it ?
    x_FL = np.zeros((res, res, res, 3))
    q = self.robot.get_q()
    for i in range(res):
        for j in range(res):
            for k in range(res):
                q[index*3:(index + 1)*3] = q_range_mesh[i, j, k] # replace mesh in corresponding in joints
                x_FL[i, j, k] = self.robot.fk_pose(q = q, EE_name = EE_name)
                self.visualize_point(x_FL[i, j, k], rgba=rgba)

  def velocity_profile(self, num_time_steps, full_duration, current, desired):
    dt = full_duration/num_time_steps

    # Define your objective function to minimize jerk
    def objective_function(vars):
      smoothness_measure = 0
      for i in range(12): #Get sum of jerk for each joint movement
        velocities = vars[i * num_time_steps : (i+1) * num_time_steps]
        jerk = np.diff(velocities, axis=0, prepend=0, append=0, n=2)
        smoothness_measure += np.sum(np.abs(jerk))
      return smoothness_measure

    list_of_constraints = [] 
    # define all 12 of position constraints function for each joint
    for i in range(12):
        def constraint(vars, i = i): #https://stackoverflow.com/questions/3431676/creating-functions-or-lambdas-in-a-loop-or-comprehension
            velocities = vars[i * num_time_steps : (i+1) * num_time_steps]
            return current[i] + np.sum(velocities * dt) - desired[i] # Ensure joint reaches target position
        list_of_constraints.append(constraint)

    # Set up the list of nonlinear inequality constraint dictionaries
    constraints = []
    for i in range(12):
        constraints.append({'type': 'eq', 'fun': list_of_constraints[i]})

    # Initial guess for velocities and duration
    initial_guess = np.zeros(12 * num_time_steps)  # Adjust based on your requirements

    # Run the optimization
    result = minimize(fun=objective_function, x0=initial_guess, method='SLSQP', constraints=constraints)
    print(result)

    # Extract the optimal velocities
    optimal_velocities = []
    for i in range(12):
        optimal_velocities.append(result.x[i * num_time_steps : (i+1) * num_time_steps])
        #print(f'optimal_velocities of {name_joints[i]}: {optimal_velocities[i]}')
        print(f'{list_of_constraints[i](result.x)}')

    return optimal_velocities

  def x_des(self, target):
    """
    Defines the x_des dictionary to define all the desired positions of the front EEs whilst keeping the Hind legs put

    Args:
        target: set of points of desired EE positions

    Returns:
    x_des: dictionary of the desired EEs positions
    """
    x_des ={'FL_FOOT': target['pos']["left"],
        'FR_FOOT': target['pos']["right"],
        'HL_FOOT': self.robot.fk_pose(q=self.robot.get_q(), EE_name="HL_FOOT"), #keep current EE position for Hind Legs
        'HR_FOOT': self.robot.fk_pose(q=self.robot.get_q(), EE_name="HR_FOOT")} #keep current EE position for Hind Legs
    
    return x_des

  def speed_animate(self, q_2, q_1 = None, t_max = 3, num_time_steps = 10, timed=False):
      """
      Animates a transition between two robot joint configurations with a velocity profile.

      Args:
        q_1: Starting joint positions.
        q_2: Ending joint positions.
        t_max: Maximum simulation time.
        num_time_steps: number of time steps within the animation, each step with a different set of joint velocities
        timed: Whether to print the simulation time (optional).
      """


      dt = t_max/num_time_steps

      if q_1 is None:
        q_1 = self.robot.get_q()

      optimal_velocities = self.velocity_profile(num_time_steps, t_max, q_1, q_2) # get optimal velocities for each joint

      if timed:
        start_time = time.time()

      q = q_1
      for i in range(num_time_steps):
        velocities = [sub_array[i] for sub_array in optimal_velocities]
        q += velocities
        self.animate(q_2=q, t_max = dt, dt = dt/100)


      if timed:
        end_time = time.time()
        simulation_time = end_time - start_time
        print(f"Simulation time: {simulation_time:.2f} seconds")

  def TOPPRA(self, points):
    """
    Defines the x_des dictionary to define all the desired positions of the front EEs whilst keeping the Hind legs put

    Args:
        N_samples: Number of points in the path
        points: array of the different via_points of the EE's path

    Returns:
    x_des: dictionary of the desired EEs positions
    """

    dof = 12
    way_pts = []
    N_samples = len(points)
    ss = np.linspace(0, 1, N_samples)
    vlims = np.ones(dof)*4 #Velocity constraints defined to be 4 rad/s
    alims = np.ones(dof)*5 #Acceleration constraints defined to be 5 rad/s^2

    for i in range(N_samples):
      x_des = self.x_des(target = points[i])
      q = self.inverse_kinematics_adjusted(x_des, q_ref = points[i]['q_ref']) # Use the appropriate q_ref for the IK computation
      way_pts.append(q.tolist())

    # Define the geometric path and two constraints.
    path = ta.SplineInterpolator(ss, way_pts)
    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(alims)

    # We solve the parametrization problem using the
    # `ParametrizeConstAccel` parametrizer. This parametrizer is the
    # classical solution, guarantee constraint and boundary conditions
    # satisfaction.
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
    jnt_traj = instance.compute_trajectory()

    ################################################################################
    # The output trajectory is an instance of
    # :class:`toppra.interpolator.AbstractGeometricPath`.
    # ts_sample = np.linspace(0, jnt_traj.duration, 100)
    ts_sample = np.arange(0, jnt_traj.duration, self.dt_control)
    qs_sample = jnt_traj(ts_sample) # ("Position (rad)")
    #qds_sample = jnt_traj(ts_sample, 1) #("Velocity (rad/s)")
    #qdds_sample = jnt_traj(ts_sample, 2) #("Acceleration (rad/s2)")
    ################################################################################

    return jnt_traj.duration, qs_sample

  def TOPPRA_speed_animate(self, points, timed=False, ctrl = True):
      """
      Animates a transition between two robot joint configurations with a velocity profile.

      Args:
        points: array of the different via_points for the given movement
      """

      N_samples = len(points)
      duration, optimal_path = self.TOPPRA(points)
      num_time_steps = len(optimal_path)

      q = self.robot.get_q()

      if timed:
        start_time = time.time()

      for i in range(num_time_steps):
        q = optimal_path[i]
        self.robot.set_q(q, ctrl)
        if ctrl:
          for _ in range(3): # When sending a position control command, give time for it to actually reach the command
            self.robot.step()
        else:
          self.robot.forward()
        if self.v is not None:
          self.v.sync()

      if timed:
        end_time = time.time()
        simulation_time = end_time - start_time
        print(f'optimal duration of movement: {duration}')
        print(f'time for given momvement: {simulation_time}')

  def simulate(self, steps=500):# 1000 steps => 4.3 - 5 seconds
    # Simulate the environment for a little
    for _ in range(steps):
        self.robot.step()
        self.v.sync()

