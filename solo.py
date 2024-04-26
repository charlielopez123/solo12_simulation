import mujoco
import numpy as np
#import quaternion
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
            self.set_q(q_init)

        self.nu = self.model.nu
        self.qmin = np.pi * np.array([-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,])#np.pi * np.ones(self.ndof) #self.model.actuator_ctrlrange[:,0]
        self.qmax =  np.pi * np.array([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]) #np.pi * np.ones(self.ndof)  #self.model.actuator_ctrlrange[:,1] # np.pi * np.array([0, 1, 1, 0, 1, -1, 0, -1, -1, 0, -1, -1,])


        self.bounds = Bounds(self.qmin, self.qmax)
        self.q_home = np.zeros(self.ndof) # home position of the robot
        self.eps = 1e-6 # tolerance for the constraints of the inverse kinematics

        self.FR_name = 'FR_FOOT' # name of the end-effector
        
    def set_q(self, q):
        self.data.qpos[:len(q)] = q.copy()

    def get_q(self):
        return self.data.qpos[:self.ndof].copy()

    def forward(self):
        mujoco.mj_forward(self.model, self.data)
        
    def step(self):
        mujoco.mj_step(self.model, self.data)
    
    def fk_pose(self, q=None, EE_name=None):
        if q is not None:
            data_copy = copy.copy(self.data)
            data_copy.qpos[:len(q)] = q
            mujoco.mj_fwdPosition(self.model, data_copy)
            data = data_copy
        else:
            data = self.data
        if EE_name is None:
            EE_name = self.FR_name
        
        body = data.body(EE_name)
        x = body.xpos.copy() # position of the EE (3D vector)
        R = np.reshape(body.xmat.copy(), (3,3)) # rotation matrix of the EE (3x3 matrix)

        return x, R
    
    def fk_jac(self, q=None, EE_name=None):
        if q is not None: 
            data_copy = copy.copy(self.data)
            data_copy.qpos[:len(q)] = q
            # self.data.qpos[:7] = q[:7]
            mujoco.mj_fwdPosition(self.model, data_copy)
            data = data_copy
        else:
            data = self.data
        jac_pos = np.empty((3, self.model.nv), dtype=data.qpos.dtype)
        jac_rot = np.empty((3, self.model.nv), dtype=data.qpos.dtype)

        if EE_name is None:
            EE_name = self.FR_name
        body = data.body(EE_name)
        mujoco.mj_jacBody(self.model, data, jac_pos, jac_rot, body.id)
        
        jac = np.concatenate((jac_pos[:, :len(q)], jac_rot[:, :len(q)]), axis=0)
        return jac
    
    def inverse_kinematics(self, x_des, q_ref=None, EE_name= None, noise = np.random.normal(size=12)*0.1):
        if EE_name is None:
            EE_name = self.FR_name

        if q_ref is None: # set joint reference position
            q_ref = self.q_home
        
        def fun_and_grad(q): #  cost function and gradient
            c = 0.5 * np.sum((q-q_ref)**2)
            dc = q-q_ref
            return c, dc
        
        def con_pos(q): #position constraint
            #print(EE_name) #testing the value of EE_name
            x, _ = self.fk_pose(q, EE_name)
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
                       options={'maxiter': 100, 'ftol': 1e-6, 'disp': True})
        return res.x, res.success

