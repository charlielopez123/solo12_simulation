import numpy as np

#List of the different EEs in order
EEs = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']

# Corresponding joints for each End-Effector
EE_joints = {'FL_FOOT': [0,1,2], 'FR_FOOT': [3,4,5], 'HL_FOOT': [6,7,8], 'HR_FOOT': [9,10,11]}

# name of all the joints in the same order of the `Robot.data.qpos[:Robot.ndof]`
name_joints = ['FL_HAA', 'FL_HFE', 'FL_KFE', 'FR_HAA', 'FR_HFE', 'FR_KFE', 'HL_HAA', 'HL_HFE', 'HL_KFE', 'HR_HAA', 'HR_HFE', 'HR_KFE']

# General reference used for IK Computations
q_ref= np.array([ (1/12)*np.pi, (5/4) * np.pi,  -1/6 *np.pi,
             -(1/12)*np.pi,  (5/4) * np.pi,  -1/6 *np.pi,
                0, -np.pi/2,  np.pi,
                0, -np.pi/2,  np.pi])

# Define dictionary to store named robot poses that can be used as reference point for the inverse kinematics function
robot_poses = {
    #Initial rest pose
    "q_init": np.array([0, np.pi/2,  np.pi, 
                        0, np.pi/2,  np.pi,
                        0, 3*np.pi/2,  np.pi,
                        0, 3*np.pi/2,  np.pi]),

    "q_arms_up_halfway": np.array([ 5.95874546e-06,  2.11708770e+00,  2.04899798e+00,
                                -5.95798805e-06, 2.11708771e+00,  2.04899797e+00,
                                0.00000000e+00, -1.57079633e+00, 3.14159265e+00,
                                0.00000000e+00, -1.57079633e+00,  3.14159265e+00]),

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
                                2.48085718e-04, -1.57092560e+00,  3.14168012e+00]),

    "q_box_on_back": np.array([ 0.32631274,  1.4666653 ,  1.45947616,
                                -0.32631274,  1.4666653 ,1.45947616,
                                0.        , -1.57079633,  3.14159265,
                                0.        ,-1.57079633,  3.14159265])
}

key_points = {

#Lift arms up
    "initial_pos": {'pos': {"left": [0.1946    , 0.14795   , 0.03],
                        "right": [ 0.1946    , -0.14795   ,  0.03]}, 
                        'rgba': [0.5, 0.2, 0.3, 0.5],
                        'q_ref': robot_poses["q_init"]
                },
    
    "arms_up_halfway": {'pos': {"left": [0.1946    , 0.14795   , 0.2],
                        "right": [ 0.1946    , -0.14795   ,  0.2]}, 
                        'rgba': [0.5, 0.2, 0.3, 0.5],
                        'q_ref': robot_poses["q_arms_up_halfway"]
                },

    "arms_up": {'pos': {"left": [0.1946    , 0.14795   , 0.35299013],
                        "right": [ 0.1946    , -0.14795   ,  0.35299013]}, 
                        'rgba': [0.5, 0.2, 0.3, 0.5],
                        'q_ref': robot_poses["q_arms_up_halfway"]
                },

#Get to box contact points
    "via_point1_get2box":{'pos': {"left": [0.3, 0.12, 0.3],
                                "right": [ 0.3, -0.12,  0.3]},
                                'rgba': [1, 0, 0, 0.5],
                                'q_ref': q_ref
                },

    "via_point2_get2box":{'pos': {"left": [0.4, 0.11, 0.25],
                        "right": [ 0.4, -0.11,  0.25]},
                        'rgba': [1, 0, 0, 0.5],
                        'q_ref': q_ref
    },

    "box_high": {'pos': {"left": [0.45, 0.1, 0.15],
                "right": [ 0.45, -0.1,  0.15]},
                'rgba': [0, 1, 0, 0.5],
                'q_ref': q_ref
    },

#Lift the box up
    "via_point1_lift_box": {'pos': {"left": [0.4, 0.1, 0.21],
                        "right": [ 0.4, -0.1,  0.21]},
                        'rgba': [0, 1, 0, 0.5],
                        'q_ref': q_ref
    },

    "via_point2_lift_box": {'pos': {"left": [0.3, 0.1, 0.26],
                        "right": [ 0.3, -0.1,  0.26]},
                        'rgba': [0, 1, 0, 0.5],
                        'q_ref': q_ref
    },

    "box_up": {'pos': {"left": [0.15, 0.1, 0.3],
                "right": [0.15, -0.1,  0.3]},
                'rgba': [0, 1, 0, 0.5],
                'q_ref': q_ref
                },

#Bring box back down
    "box_on_back_halfway":{'pos':{"left": [0.05, 0.1, 0.033 + 0.25],
                "right": [0.05, -0.1,  0.033 + 0.25]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses["q_box_on_back"]
                },

    "box_on_back":{'pos':{"left": [0, 0.1, 0.033 + 0.15],
                "right": [0, -0.1,  0.033 + 0.15]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses["q_box_on_back"]
                }
}

key_movements = {

    "lift_arms": [
    key_points["initial_pos"],
    key_points["arms_up_halfway"],
    key_points["arms_up"]
    ],

    "get2box": [
    key_points["arms_up"],
    key_points["via_point1_get2box"],
    key_points["via_point2_get2box"],
    key_points["box_high"]
    ],

    "lift_box": [
    key_points["box_high"],
    key_points["via_point1_lift_box"],
    key_points["via_point2_lift_box"],
    key_points["box_up"]
    ],

    "box2back": [
    key_points["box_up"],
    key_points['box_on_back_halfway'],
    key_points["box_on_back"]
    ],

}