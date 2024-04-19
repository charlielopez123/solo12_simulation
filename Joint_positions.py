import numpy as np

EE_joints = {'FL_FOOT': [0,1,2], 'FR_FOOT': [3,4,5], 'HL_FOOT': [6,7,8], 'HR_FOOT': [9,10,11]}

# Define dictionary to store named robot poses
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
    "box_up": {"left": [0.15061844, 0.06616774, 0.35240609],
                "right": [0.15061846, -0.06616772,  0.35240609]},
    
    "box": {"left": [0.5, 0.06, 0.05],
            "right": [0.5, -0.06,  0.05]
    }
}
