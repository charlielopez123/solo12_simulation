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
from Joint_positions import *
class SoloSim:
  """
  A class for simulating a Solo robot in a MuJoCo environment.
  """

  def __init__(self,
            q_init=None,
            launch_viewer = True,
            manipulation_task = True,
            use_hind_legs = False,
            use_box = False):
    """
    Initializes the simulation environment for the Solo robot.

    Args:
        q_init (array, optional): Initial joint positions for the robot. Defaults to None.
        launch_viewer (bool, optional): Flag to launch the MuJoCo viewer. Defaults to True.
        manipulation_task (bool, optional): Flag to indicate if the simulation is for a manipulation task. If True, inverse kinematics for hind legs will be skipped. Defaults to True.
        use_hind_legs (bool, optional): Flag to indicate whether to use hind legs for manipulation tasks. Defaults to False.
        use_box (bool, optional): Flag to indicate whether to add the box for manipulation tasks. Defaults to False.
    """
    # Load scene and model
    if use_box:
      self.model = mujoco.MjModel.from_xml_path('scene.xml')
    else:
      self.model = mujoco.MjModel.from_xml_path('scene_without_box.xml')
    self.robot = Robot(self.model, q_init)

    # Launch viewer
    if launch_viewer:
      self.v = viewer.launch_passive(self.robot.model, self.robot.data)
    else: 
      self.v = None

    #Mujoco time step when using mj_step()
    self.dt_control = self.model.opt.timestep

    #If in a context of a manipulation task, skip the IK computation of the hind legs
    self.manipulation_task = manipulation_task

    #Choose which legs to use for manipulation task
    self.use_hind_legs = use_hind_legs

    #Define the velocity limits (rad/s) and acceleration limits
    self.vlim = 0.5
    self.alim = 3

    #Define the control rate with the time step
    self.dt = 0.001

  def animate(self, q_2, q_1 = None, t_max = 2, dt = 0.01, timed=False, ctrl = False):
      """
    Animates a transition between two robot joint configurations.

    Args:
      q_2 (array): Ending joint positions.
      q_1 (array, optional): Starting joint positions. Defaults to current positions if None.
      t_max (float, optional): Maximum simulation time. Defaults to 2 seconds.
      dt (float, optional): Time step for simulation. Defaults to 0.01 seconds.
      timed (bool, optional): Flag to print the simulation time. Defaults to False.
      ctrl (bool, optional): Whether to simulate the physics or not (using step() or forward()). Defaults to True.
    """
      if q_1 is None:
        q_1 = self.robot.get_q()
      if timed:
        start_time = time.time()
      for t in np.arange(0, t_max, dt):
        self.robot.set_q(t / t_max * (q_2 - q_1) + q_1, ctrl=False)
        if ctrl is False:
          self.robot.forward()
        else:
          self.robot.step()
        if self.v is not None:
          self.v.sync()
        sleep(dt)
      if timed:
        end_time = time.time()
        simulation_time = end_time - start_time
        print(f"Simulation time: {simulation_time:.2f} seconds")

  def get_front_hand_positions(self):
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

  def get_hind_hand_positions(self):
      """
      Retrieves the positions of the robot's hind "hands".

      Returns:
        A dictionary containing the positions of the HL_FOOT and HR_FOOT.
      """
      positions = {}
      # HL_FOOT Position
      x = self.robot.fk_pose(q=self.robot.get_q(), EE_name="HL_FOOT")
      positions["HL_FOOT"] = x
      # HR_FOOT Position
      x = self.robot.fk_pose(q=self.robot.get_q(), EE_name="HR_FOOT")
      positions["HR_FOOT"] = x
      return positions

  def visualize_point(self, key_pos, rgba=np.array([0.8, 0.3, 0.3, 0.5]), verbose=False):
      """
      Visualizes a point in the MuJoCo viewer.

      Args:
        key_pos: The position of the point to visualize.
        rgba (optional): The color and transparency of the point. Defaults to [0.8, 0.3, 0.3, 0.5].
        verbose (optional): If True, prints additional information. Defaults to False.
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

  def inverse_kinematics_adjusted(self, x_des, q_ref=None, q=None):
      """
      Computes the inverse kinematics for a desired end-effector position.

      Args:
        x_des (dict): Desired positions of the end-effectors in Cartesian space (x, y, z).
        q_ref (array, optional): Reference joint positions for the inverse kinematics. Defaults to current positions.
        q (array, optional): Current joint positions. Defaults to current positions.

      Returns:
        array: Joint positions to achieve the desired end-effector positions.
      """
      if q_ref is None:
        q_ref = self.robot.get_q()
      if q is None:
        q = self.robot.get_q()

      noise = self.build_noise()

      if not self.use_hind_legs:
        EE_list = ['FL_FOOT', 'FR_FOOT']
      else:
        EE_list = ['HL_FOOT', 'HR_FOOT']
      
      if self.manipulation_task: #If a manipulation task, only compute IK for front hands
        for EE in EE_list:
          q_ik, success = self.robot.inverse_kinematics(x_des[EE], q_ref=q_ref, EE_name=EE, noise=noise)
          print(f"For {EE}: {success}")
          for i in EE_joints[EE]:
            q[i] = q_ik[i]
      else:
        for EE in EE_joints:
          q_ik, success = self.robot.inverse_kinematics(x_des[EE], q_ref=q_ref, EE_name=EE, noise=noise)
          print(f"For {EE}: {success}")
          for i in EE_joints[EE]:
            q[i] = q_ik[i]

      return q

  def all_the_points(self, set_of_points = key_points):
    """
    Visualizes all the points in the viewer.

    Args:
      set_of_points (dict): Dictionary containing the points to visualize and their properties.
    """
    for situation in set_of_points:
      rgba = set_of_points[situation]['rgba']
      for EE in set_of_points[situation]['pos']:
        self.visualize_point(set_of_points[situation]['pos'][EE], rgba)

  def get_joint_positions(self, EE):
    """
    Prints the positions of the chosen end-effector's corresponding joints.

    Args:
      EE (str): Chosen end-effector name.
    """

    q = self.robot.get_q()
    for joint in EE_joints[EE]:
      print(f"Joint {name_joints[joint]} Position: {q[joint]}")

  def build_noise(self):
    """
    Builds a noise array to add to the x0 initial solution of the IK solver.
    We want a symmetric noise as to converge to a symmetric solution for both symmetric EE

    Returns:
      array: Noise array to ensure symmetric solutions for end-effectors.
    """

    # Generate three random normal numbers for additional noise given to an EE
    random_numbers = np.random.normal(size=3) # [a, b, c]
    random_numbers_negative = random_numbers * np.array([-1, 1, 1]) # [-a, b, c]

    pattern = np.concatenate((random_numbers, random_numbers_negative)) # [a, b, c, -a, b, c]
    #Build the noise array to add to each of the initial solution of the solver
    noise = np.tile(pattern, 2)*0.001 # [a, b, c, -a, b, c, a, b, c, -a, b, c]

    return noise

  def workspace(self, res = 10, rgba = np.array([0, 0, 1, 0.8]), EE_name = 'FL_FOOT'):
    """
    Visualizes a cloud of possible end-effector positions to estimate the workspace.

    Args:
      res (int, optional): Resolution of joint positional increments. Defaults to 10.
      rgba (array, optional): RGBA color and opacity of workspace points. Defaults to blue.
      EE_name (str, optional): Chosen end-effector name. Defaults to 'FL_FOOT'.
    """
    # q_min = np.zeros(12)
    # q_max = 2*np.pi * np.ones(12)
    q_min = np.array([np.deg2rad(-75), -np.pi, -np.pi, np.deg2rad(-255), -np.pi, -np.pi, np.deg2rad(-75), -np.pi, -np.pi, np.deg2rad(-255), -np.pi, -np.pi])
    q_max = np.array([np.deg2rad(255),  np.pi,  np.pi, np.deg2rad(75),    np.pi,  np.pi, np.deg2rad(255), np.pi, np.pi, np.deg2rad(75), np.pi, np.pi])

    q_range = np.linspace(0, 2*np.pi, res) # (10, 3)
    q_range_0= np.linspace(q_min[0], q_max[0], res) # (10, 3)
    q_range_1= np.linspace(q_min[1], q_max[1], res) # (10, 3)
    q_range_2= np.linspace(q_min[2], q_max[2], res) # (10, 3)

    q_range_mesh = np.array(np.meshgrid(q_range_0, q_range_1, q_range_2)) # (3, 10, 10, 10)
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

  def velocity_profile(self, dt, full_duration, current, desired, plot = False, verbose = False):
    """
        Deprecated, more optimal using TOPPRA
        Generates an optimal velocity profile to minimize jerk for joint movements.

        Args:
          dt (float): Time step in seconds.
          full_duration (float): Total duration of the movement.
          current (array): Initial joint positions.
          desired (array): Desired joint positions.
          plot (bool, optional): If True, plots the joint positions, velocities, and accelerations over time. Defaults to False.
          verbose (bool, optional): If True, prints additional information. Defaults to False.

        Returns:
          tuple: A tuple containing:
            - array: Time-discretized joint positions following the optimal velocity profile.
            - array: Time-discretized joint velocities following the optimal velocity profile.
            - array: Time-discretized joint accelerations following the optimal velocity profile.
        """
    num_time_steps = int(full_duration/dt)

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

    # Initial guess: interpolate positions, zero velocities
    initial_positions = np.linspace(current, desired, num=num_time_steps).flatten()
    initial_velocities = np.zeros(12 * num_time_steps)
    initial_guess = np.concatenate([initial_positions, initial_velocities])

    # Check the objective function value at the initial guess
    print(f'Initial objective function value: {objective_function(initial_guess)}')

    # Run the optimization
    result = minimize(fun=objective_function, x0=initial_guess, method='SLSQP', constraints=constraints)

    pos = []
    vel = []
    acc = []
    # Extract and print the optimal positions and velocities
    if result.success:
          for i in range(12):
              optimal_positions = result.x[i * num_time_steps: (i + 1) * num_time_steps]
              optimal_velocities = result.x[12 * num_time_steps + i * num_time_steps: 12 * num_time_steps + (i + 1) * num_time_steps]
              optimal_accelerations = np.diff(optimal_velocities, prepend=0) / dt
              if verbose:
                print(f'Optimal positions of {name_joints[i]}: {optimal_positions}')
                print(f'Optimal velocities of {name_joints[i]}: {optimal_velocities}')
                print(f'Optimal accelerations of {name_joints[i]}: {optimal_accelerations}')
              if plot:
                pos.append(optimal_positions)
                vel.append(optimal_velocities)
                acc.append(optimal_accelerations)
    else:
        print("Optimization failed:", result.message)

    if plot:
      return np.array(pos), np.array(vel), np.array(acc)

  def x_des(self, target):
    """
    Defines the x_des dictionary to define all the desired positions of the front EEs whilst keeping the Hind legs put

    Args:
        target: set of points of desired EE positions

    Returns:
    x_des: dictionary of the desired EEs positions
    """
    if not self.use_hind_legs:
      x_des ={'FL_FOOT': target['pos']["left"],
          'FR_FOOT': target['pos']["right"],
          'HL_FOOT': self.robot.fk_pose(q=self.robot.get_q(), EE_name="HL_FOOT"), #keep current EE position for Hind Legs
          'HR_FOOT': self.robot.fk_pose(q=self.robot.get_q(), EE_name="HR_FOOT")} #keep current EE position for Hind Legs
    else:
      x_des ={'FL_FOOT': self.robot.fk_pose(q=self.robot.get_q(), EE_name="FL_FOOT"),
          'FR_FOOT': self.robot.fk_pose(q=self.robot.get_q(), EE_name="FR_FOOT"),
          'HL_FOOT': target['pos']["left"], #keep current EE position for Hind Legs
          'HR_FOOT': target['pos']["right"]} #keep current EE position for Hind Legs
    return x_des

  def speed_animate(self, q_2, q_1 = None, t_max = 3, dt = 0.01, timed=False):
      """
      Deprecated, more optimal to use TOPPRA trajectory profiles
      Animates a transition between two robot joint configurations with a velocity profile.

      Args:
        q_1: Starting joint positions.
        q_2: Ending joint positions.
        t_max: Maximum simulation time.
        num_time_steps: number of time steps within the animation, each step with a different set of joint velocities
        timed: Whether to print the simulation time (optional).
      """


      num_time_steps = t_max/dt
      print(type(num_time_steps))

      if q_1 is None:
        q_1 = self.robot.get_q()

      optimal_positions = self.velocity_profile(dt, t_max, q_1, q_2) # get optimal velocities for each joint

      if timed:
        start_time = time.time()

      q = q_1
      for i in range(num_time_steps):
        positions = [sub_array[i] for sub_array in optimal_positions]
        q = positions
        self.animate(q_2=q, t_max = dt, dt = dt/100)


      if timed:
        end_time = time.time()
        simulation_time = end_time - start_time
        print(f"Simulation time: {simulation_time:.2f} seconds")

  def TOPPRA(self, points):
    """
    Given an array of points in Cartesian space (x, y, z) that the EE has to follow: [start, ..., via_points, ..., end]. 
    This function returns the optimal trajectories for the joints to follow the given points path with the starting point, 
    end point and every via point in between for the movement.

    Args:
      points: array of the different via_points of the EE's path in in Cartesian space (x, y, z)

    Returns:
      duration: duration of the given movement
      ts_sample: time at each time step
      qs_sample: array of all the different joint configurations at each time step  from start to finish
      qds_sample: array of all the different joint speeds at each time step from start to finish
      qdds_sample: array of all the different joint accelerations at each time step from start to finish
    """

    dof = 12
    way_pts = []
    N_samples = len(points)
    ss = np.linspace(0, 1, N_samples)
    vlims = np.ones(dof)*self.vlim #Velocity constraints defined to be 4 rad/s
    alims = np.ones(dof)*self.alim #Acceleration constraints defined to be 5 rad/s^2

    way_pts.append(self.robot.get_q().tolist()) #Skip the IK computation of the starting as it is just the actual current position
    for i in range(1, N_samples): #Start IK computation from the first via_point
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
    duration = jnt_traj.duration
    ts_sample = np.arange(0, jnt_traj.duration, self.dt)
    qs_sample = jnt_traj(ts_sample) # ("Position (rad)")
    qds_sample = jnt_traj(ts_sample, 1) #("Velocity (rad/s)")
    qdds_sample = jnt_traj(ts_sample, 2) #("Acceleration (rad/s2)")
    ################################################################################

    return duration, qs_sample, qds_sample, qdds_sample

  def TOPPRA_speed_animate(self, points, timed=False, ctrl = True, plot = False):
      """
    Animates a transition between two robot joint configurations with a velocity profile
    generated using the TOPPRA algorithm.

    Args:
        points (list): Array of via_points for the movement in Cartesian space (x, y, z).
        timed (bool, optional): Whether to time the movement animation using time.time(). Defaults to False.
        ctrl (bool, optional): Whether to simulate the physics or not (using step() or forward()). Defaults to True.
        plot (bool, optional): Whether to return the trajectories of positions, velocities, and accelerations
                               of the joints for later plotting. Defaults to False.

    Returns:
        If plot is True, returns a tuple (duration, pos_sim, pos_TOPPRA, vel, acc):
            duration (float): The duration of the movement.
            pos_sim (np.ndarray): Array of joint positions at each time step during the simulation.
            pos_TOPPRA (np.ndarray): Array of optimal joint positions at each time step generated by TOPPRA.
            vel (np.ndarray): Array of joint velocities at each time step generated by TOPPRA.
            acc (np.ndarray): Array of joint accelerations at each time step generated by TOPPRA.
    """

      pos_sim = [] #initialize empty array to fill with q_joints at different time_steps using get_q()
      pos_TOPPRA = []
      vel = []
      acc = []

      N_samples = len(points)
      duration, optimal_path, optimal_vel, optimal_acc = self.TOPPRA(points)
      print(f'duration of the movement: {duration}')
      num_time_steps = len(optimal_path)

      pos_TOPPRA.append(optimal_path)
      vel.append(optimal_vel)
      acc.append(optimal_acc)

      
      q = self.robot.get_q()

      if timed:
        start_time = time.time()

      for i in range(num_time_steps):
        q = optimal_path[i]
        self.robot.set_q(q, ctrl)
        if ctrl:
          for _ in range(1): # When sending a position control command, give time for it to actually reach the command
            self.robot.step()
        else:
          self.robot.forward()
        if self.v is not None:
          self.v.sync()
        pos_sim.append(self.robot.get_q())



      if timed:
        end_time = time.time()
        simulation_time = end_time - start_time
        print(f'optimal duration of movement: {duration}')
        print(f'time for given movement: {simulation_time}')

      if plot:
        return duration, np.array(pos_sim), np.squeeze(np.array(pos_TOPPRA)), np.squeeze(np.array(vel)), np.squeeze(np.array(acc))

  def simulate(self, steps=500):# 1000 steps => 4.3 - 5 seconds
    """
    Simulates the robot's environment for a specified number of steps.

    Args:
        steps (int, optional): The number of simulation steps to execute. Defaults to 500.
    """
    for _ in range(steps):
        self.robot.step()
        self.v.sync()

