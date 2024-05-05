import numpy as np

# Corresponding joints for each End-Effector
EE_joints = {'FL_FOOT': [0,1,2], 'FR_FOOT': [3,4,5], 'HL_FOOT': [6,7,8], 'HR_FOOT': [9,10,11]}

# name of all the joints in the same order of the `Robot.data.qpos[:Robot.ndof]`
name_joints = ['FL_HAA', 'FL_HFE', 'FL_KFE', 'FR_HAA', 'FR_HFE', 'FR_KFE', 'HL_HAA', 'HL_HFE', 'HL_KFE', 'HR_HAA', 'HR_HFE', 'HR_KFE']

q_ref= np.array([ (1/12)*np.pi, (5/4) * np.pi,  -1/6 *np.pi,
             -(1/12)*np.pi,  (5/4) * np.pi,  -1/6 *np.pi,
                0, -np.pi/2,  np.pi,
                0, -np.pi/2,  np.pi])
# Define dictionary to store named robot poses that can be used as reference point for the inverse kinematics function
robot_poses = {
    #Initial rest pose
    "q_init": np.array([0, np.pi/2,  np.pi, 
                        0, np.pi/2,  np.pi,
                        0, -np.pi/2,  np.pi,
                        0, -np.pi/2,  np.pi]),

    #Front arms up
    "q_arms_up_straight": np.array([0, np.pi,  0, 
                        0, np.pi,  0,
                        0, -np.pi/2,  np.pi,
                        0, -np.pi/2,  np.pi]),

    # Place front feet on either side of the box
    "q_hands_on_box_elbows_bent_under": np.array([np.pi/4, 3 * np.pi/2,  -np.pi/12,
                                                -np.pi/4, 3 * np.pi/2,  -np.pi/12,
                                                0, -np.pi/2,  np.pi,
                                                0, -np.pi/2,  np.pi]),

    # Place front feet on either side of the box
    "q_hands_on_box_elbows_bent_sideways": np.array([(1/2) * np.pi, -1/2 * np.pi,  -(1/12) * np.pi, 
                                                    -(1/2) * np.pi, -1/2 * np.pi,  -(1/12) * np.pi,
                                                    0, -np.pi/2,  np.pi,
                                                    0, -np.pi/2,  np.pi]),


    # Final Position of Box in the air
    "q_hold_box_up": np.array([ 1.82599084e-01,  3.57812781e+00, -1.19878120e+00,
                                -1.82599084e-01, 3.57812781e+00, -1.19878120e+00,
                                -2.48085718e-04, -1.57092560e+00,3.14168012e+00,
                                2.48085718e-04, -1.57092560e+00,  3.14168012e+00])
}

key_points = {

    # left and right points of contact with the supposed box when held up
    
    "arms_up": {"left": [0.1946    , 0.14795   , 0.35299013],
                "right": [ 0.1946    , -0.14795   ,  0.35299013]},

    "box_high": {"left": [0.45, 0.1, 0.15],
                "right": [ 0.45, -0.1,  0.15]
    },

    "via_point1_lift_box": {"left": [0.4, 0.1, 0.21],
                        "right": [ 0.4, -0.1,  0.21]
    },

    "via_point2_lift_box": {"left": [0.3, 0.1, 0.26],
                        "right": [ 0.3, -0.1,  0.26]
    },

    "box_up": {"left": [0.15, 0.1, 0.3],
                "right": [0.15, -0.1,  0.3]},

    "box_on_back":{"left": [0, 0.1, 0.033 + 0.15],
                "right": [0, -0.1,  0.033 + 0.15]}
}