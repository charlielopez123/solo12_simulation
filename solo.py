"""
This code was adapted from Julius Jankowski's original code.

This script defines a class `Robot` for handling robotic manipulator operations, including forward and inverse kinematics, joint control, and optimization for inverse kinematics.

The `Robot` class includes the following methods:

1. __init__(self, model, q_init=None): Initializes the Robot object with a Mujoco model and optional initial joint positions.

2. set_q(self, q, ctrl=True): Sets the joint positions or control commands.

3. get_q(self): Returns the current joint positions.

4. forward(self): Performs forward kinematics computation, without consideration for dynamics and physics.

5. step(self): Moves the simulation forward by one timestep, with consideration for dynamics and physics.

6. fk_pose(self, q=None, EE_name=None): Computes the forward kinematics pose of the end-effector.

7. fk_jac(self, q=None, EE_name=None): Computes the forward kinematics Jacobian.

8. inverse_kinematics(self, x_des, q_ref=None, EE_name=None, noise=np.random.normal(size=12)*0.1): Computes inverse kinematics to find joint positions for a desired end-effector pose.

    - The cost function minimizes the squared difference between current and desired joint positions.
    - Positional constraints ensure that the end-effector reaches the desired pose.
    - The optimization is performed using the Sequential Least Squares Programming (SLSQP) method.
    - The initial solution includes noise to avoid singularities.

"""


import mujoco
import numpy as np
from scipy.optimize import minimize, Bounds, NonlinearConstraint
import copy

class Robot:
    def __init__(
      self,
      model,
      q_init=None
    ):
        self.ndof = 12

        self.model = model
        self.data = mujoco.MjData(self.model)

        if q_init is not None:
            self.set_q(q_init, ctrl=False)

        self.nu = self.model.nu
        self.qmin= -0.1 + np.array([np.deg2rad(-75), 0, -np.pi, np.deg2rad(-255), 0, -np.pi, np.deg2rad(-75), 0, -np.pi, np.deg2rad(-255), 0, -np.pi])
        self.qmax = 0.1  + np.array([np.deg2rad(255),  2*np.pi,  np.pi, np.deg2rad(75),  2*np.pi,  np.pi, np.deg2rad(255), 2*np.pi, np.pi, np.deg2rad(75), 2*np.pi, np.pi])


        self.bounds = Bounds(self.qmin, self.qmax)
        self.q_home = np.zeros(self.ndof) # home position of the robot
        self.eps = 1e-6 # tolerance for the constraints of the inverse kinematics

        self.FL_name = 'FL_FOOT' # name of the end-effector
        
    def set_q(self, q, ctrl = True):
        if not ctrl:
            self.data.qpos[7:7+12] = q.copy() # self.data.qpos of shape (26,), first 7 positions (x, y, z, quaternion) for base link, 12 for joint positions considered hinges, then 7 last for box
        else:
            self.data.ctrl = q.copy() # self.data.ctrl of shape (12,) position control command for each joint

    def get_q(self):
        return self.data.qpos[7:7+12].copy()

    def forward(self):
        mujoco.mj_forward(self.model, self.data)
        
    def step(self):
        mujoco.mj_step(self.model, self.data)
    
    def fk_pose(self, q=None, EE_name=None):
        if q is not None:
            data_copy = copy.copy(self.data)
            data_copy.qpos[7:7+12] = q
            mujoco.mj_fwdPosition(self.model, data_copy)
            data = data_copy
        else:
            data = self.data
        if EE_name is None:
            EE_name = self.FL_name
        
        body = data.body(EE_name)
        x = body.xpos.copy() # position of the EE (3D vector)
        R = np.reshape(body.xmat.copy(), (3,3)) # rotation matrix of the EE (3x3 matrix)
        return x
    
    def fk_jac(self, q=None, EE_name=None):
        if q is not None: 
            data_copy = copy.copy(self.data)
            data_copy.qpos[7:7+12] = q
            # self.data.qpos[:7] = q[:7]
            mujoco.mj_fwdPosition(self.model, data_copy)
            data = data_copy
        else:
            data = self.data
        jac_pos = np.empty((3, self.model.nv), dtype=data.qpos.dtype)
        jac_rot = np.empty((3, self.model.nv), dtype=data.qpos.dtype)

        if EE_name is None:
            EE_name = self.FL_name
        body = data.body(EE_name)
        mujoco.mj_jacBody(self.model, data, jac_pos, jac_rot, body.id)
        
        jac = np.concatenate((jac_pos[:, 6:6+12], jac_rot[:, 6:6+12]), axis=0)
        return jac
    
    def inverse_kinematics(self, x_des, q_ref=None, EE_name= None, noise = np.random.normal(size=12)*0.1):
        if EE_name is None:
            EE_name = self.FL_name

        if q_ref is None: # set joint reference position
            q_ref = self.q_home

        q_ref += 2*np.pi * (q_ref < self.qmin) #readjust the configurations to be within the bounds 
        q_ref -= 2*np.pi * (q_ref > self.qmax)

        def fun_and_grad(q): #  cost function and gradient
            c = 0.5 * np.sum((q-q_ref)**2)
            dc = q-q_ref
            return c, dc
        
        def con_pos(q): #position constraint
            #print(EE_name) #testing the value of EE_name
            x = self.fk_pose(q, EE_name)
            return x-x_des #position error
        def con_pos_jac(q):
            jac = self.fk_jac(q,EE_name)
            return jac[:3]
        nlc_pos = NonlinearConstraint(con_pos, -self.eps, self.eps, jac = con_pos_jac) # constrain the positional error between [-eps, eps]
        nlc = nlc_pos
        
        #intitial solution for the solver
        x0 = q_ref.copy() + noise #+ np.random.normal(size=12)*0.1 # noise added to initial position of the optimization problem as to avoid singularities
        res = minimize(fun_and_grad, # return c, dc: cost function and gradient
                       x0, #q_ref
                       method='SLSQP', # SLSQP is ideal for mathematical problems for which the objective function and the constraints are twice continuously differentiable.
                       jac=True, # If jac is a Boolean and is True, fun is assumed to return a tuple (f, g) containing the objective function and the gradient
                       bounds=self.bounds, #[robot.qmin, robot.qmax]
                       constraints=nlc, 
                       options={'maxiter': 100, 'ftol': 1e-6, 'disp': False})
        return res.x, res.success

