import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time


def load_env(q_init):
    #Load scene and model
    model = mujoco.MjModel.from_xml_path("scene.xml")
    robot = Robot(model, q_init)
    #Launch viewer
    v = viewer.launch_passive(robot.model, robot.data)
    return robot, model, v

# animate q change from q_1 to q_2, in t_max time with dt intervals
def animate(robot, q_1, q_2, t_max, dt, timed = False):
     if timed:
               start_time = time.time()
     for t in np.arange(0, t_max, dt):
          robot.set_q(t/t_max * (q_2 - q_1) + q_1)
          robot.step()
          v.sync()
          sleep(dt)
     if timed: 
          end_time = time.time() 
          simulation_time = end_time - start_time
          print(f"Simulation time: {simulation_time:.2f} seconds")

# returns position of FL_FOOT and FR_FOOT
def get_hand_position():
     # FL_FOOT Position
     x, _ = robot.fk_pose(q = robot.get_q(), EE_name= 'FL_FOOT')
     print(f"position of FL_FOOT: {x}")
     # FR_FOOT Position
     x, _ = robot.fk_pose(q = robot.get_q(), EE_name= 'FR_FOOT')
     print(f"position of FR_FOOT: {x}")

# visualize a point the viewer
def show_point(key_pos, id):
     # show key positions
     key_pos1 = [0.15061844, 0.06616774, 0.35240609]
     # Define sphere radius (adjust as needed)
     sphere_radius = 1
     v.user_scn.ngeom = 0
     
     # Create a new sphere geometry
     geom_id = mujoco.mjv_initGeom(
          v.user_scn.geoms[id],  # You can specify an existing geometry ID for reuse
          type=mujoco.mjtGeom.mjGEOM_SPHERE,
          size=[sphere_radius, 0, 0],
          pos=key_pos1,
          mat=np.eye(3).flatten(),  # Material properties (identity for now)
          rgba=np.array([0.8, 0.3, 0.3, 0.5]),  # Reddish color with transparency
     )
     v.user_scn.ngeom = id
     v.sync()


     # Handle potential errors (optional)
     if geom_id == -1:
          print("Error creating geometry")
     else:
          print(f"Point visualized with geometry ID: {v.user_scn.geoms[id]}")
          v.user_scn.ngeom = id
          v.sync()

#Initial rest pose
q_init = np.array([0, np.pi/2,  np.pi, 
     0, np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi])

robot, model, v = load_env(q_init)

#Front arms up
q_end = np.array([0, np.pi,  0, 
     0, np.pi,  0,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi])

animate(robot, q_init, q_end, t_max = 2, dt = 0.01, timed = True)
#position of FL_FOOT: [0.18811347 0.14797154 0.35292119]
#position of FR_FOOT: [ 0.1881135  -0.14797147  0.35292121]

# Place front feet on either side of the box
q = np.array([np.pi/4, 3 * np.pi/2,  -np.pi/12,
    -np.pi/4, 3 * np.pi/2,  -np.pi/12,
     0, -np.pi/2,  np.pi,
     0, -np.pi/2,  np.pi])

animate(robot, q_end, q, t_max = 2, dt = 0.01, timed = True)
#Hands on either side of box:
#position of FL_FOOT: [0.50831553 0.09773597 0.10973119]
#position of FR_FOOT: [ 0.50831553 -0.09773602  0.10973116]

# bring arms back
q_next = q.copy()
q_next[0] = np.pi/12
q_next[1] = np.pi
q_next[3] = -np.pi/12
q_next[4] = np.pi

animate(robot, q, q_next, t_max = 2, dt = 0.01, timed = True)
