import numpy as np

# Corresponding joints for each End-Effector
EE_joints = {'FL_FOOT': [0,1,2], 'FR_FOOT': [3,4,5], 'HL_FOOT': [6,7,8], 'HR_FOOT': [9,10,11]}

# name of all the joints in the same order of the `Robot.data.qpos[:Robot.ndof]`
name_joints = ['FL_HAA', 'FL_HFE', 'FL_KFE', 'FR_HAA', 'FR_HFE', 'FR_KFE', 'HL_HAA', 'HL_HFE', 'HL_KFE', 'HR_HAA', 'HR_HFE', 'HR_KFE']

# Define dictionary to store named robot poses that can be used as reference point for the inverse kinematics function
robot_poses = {
    #Initial rest pose
    "q_init": np.array([0, np.pi/2,  np.pi, 
                          0, np.pi/2,  np.pi,
                          0, -np.pi/2,  np.pi,
                          0, -np.pi/2,  np.pi]),

    #Front arms up
    "q_arms_up": np.array([0, np.pi,  0, 
                         0, np.pi,  0,
                         0, -np.pi/2,  np.pi,
                         0, -np.pi/2,  np.pi]),

    # Place front feet on either side of the box
    "q_hands_on_box": np.array([np.pi/4, 3 * np.pi/2,  -np.pi/12,
                             -np.pi/4, 3 * np.pi/2,  -np.pi/12,
                              0, -np.pi/2,  np.pi,
                              0, -np.pi/2,  np.pi]),

    # Final Position of Box in the air
    "q_hold_box_up": np.array([np.pi/12, np.pi,  -np.pi/12,
                             -np.pi/12, np.pi,  -np.pi/12,
                              0, -np.pi/2,  np.pi,
                              0, -np.pi/2,  np.pi]),
}

key_points = {

    # left and right points of contact with the supposed box when held up
    "box_up": {"left": [0.15061844, 0.06616774, 0.35240609],
                "right": [0.15061846, -0.06616772,  0.35240609]},
    
    # left and right points of contact with the supposed box on the ground in front of the robot
    "box": {"left": [0.49, 0.047, 0.08],
            "right": [0.49, -0.047, 0.08]
    }
}
