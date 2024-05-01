import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Joint_positions import *

class SoloSim:
  """
  A class for simulating a Solo robot in a MuJoCo environment.
  """

  def __init__(self,
             q_init=None):
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
    self.v = viewer.launch_passive(self.robot.model, self.robot.data)

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
        self.robot.set_q(t / t_max * (q_2 - q_1) + q_1)
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
      x, _ = self.robot.fk_pose(q=self.robot.get_q(), EE_name="FL_FOOT")
      positions["FL_FOOT"] = x
      # FR_FOOT Position
      x, _ = self.robot.fk_pose(q=self.robot.get_q(), EE_name="FR_FOOT")
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

# inverse kinematics function calling the inverse_kinematics function from solo.py and 
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

    for EE in EE_joints:
      q_ik, success = self.robot.inverse_kinematics(x_des[EE], q_ref = q_ref, EE_name = EE, noise = noise)
      print(f"for {EE}: {success}")
      for i in EE_joints[EE]: # change the relevant joints of the EE
        q[i] = q_ik[i]

    return q

  def visualize_all_the_points(self):
    for situation in key_points:
      #print(situation)
      for EE in key_points[situation]:
        #print(EE)
        self.visualize_point(key_points[situation][EE])

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
    noise = np.tile(pattern, 2)*0.1 # [a, b, c, -a, b, c, a, b, c, -a, b, c]

    return noise

  def visualize_workspace(self, res = 10, rgba = np.array([0, 0, 1, 0.8]), EE_name = 'FL_FOOT'):
    """
    Visualizes a cloud of possible EE positions as to get a rough estimate of the EE's workspace

    Args:
      res: resolution of joint positional increments
      rgba: rgb color and opacity of workspace points
      EE_name : Chosen EE from which we want information on the corresponding workspace
    """
    q_min = np.zeros(12)
    q_max = 2*np.pi * np.ones(12)

    q_range = np.linspace(0, 2*np.pi, res) # (10, 3)
    q_range_mesh = np.array(np.meshgrid(q_range, q_range, q_range)) # (3, 10, 10, 10)
    # Reshape to (10, 10, 10, 3)
    q_range_mesh = q_range_mesh.T # (10, 10, 10, 3)
    print(q_range_mesh)

    x_FL = np.zeros((res, res, res, 3))
    for i in range(res):
        for j in range(res):
            for k in range(res):
                x_FL[i, j, k] = self.robot.fk_pose(q = q_range_mesh[i, j, k], EE_name = EE_name)
                self.visualize_point(x_FL[i, j, k], rgba=rgba)