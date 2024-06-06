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

# General reference used for IK Computations if using hind legs
q_ref_hind_legs = np.array([ 0, np.pi/2,  np.pi, 
                        0, np.pi/2,  np.pi,
                        (1/12)*np.pi, (5/4) * np.pi - np.pi/2,  1/6 *np.pi,
                        -(1/12)*np.pi,  (5/4) * np.pi - np.pi/2,  1/6 *np.pi])

# Define dictionary to store named robot poses for the front legs that can be used as reference point for the inverse kinematics function
robot_poses = {
    #Initial rest pose
    "q_init": np.array([0, np.pi/2,  np.pi, 
                        0, np.pi/2,  np.pi,
                        0, 3*np.pi/2,  -np.pi,
                        0, 3*np.pi/2,  -np.pi]),

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

# Define dictionary to store named robot poses for the front legs that can be used as reference point for the inverse kinematics function
robot_poses_hind_legs = {
    #Initial rest pose 
    "q_init": np.array([0, np.pi/2,  np.pi, 
                        0, np.pi/2,  np.pi,
                        0, 3*np.pi/2,  -np.pi,
                        0, 3*np.pi/2,  -np.pi]),

    "q_arms_up_halfway": np.array([ 0, np.pi/2,  np.pi, 
                                0, np.pi/2,  np.pi,
                                0,  2.11708770e+00 + 2*np.pi/3,  -np.pi+np.pi/3,
                                0, 2.11708771e+00 + 2*np.pi/3,  -np.pi+np.pi/3]),

    #hind arms up
    "q_arms_up_straight": np.array([0, np.pi/2,  np.pi, 
                        0, np.pi/2,  np.pi,
                        0, np.pi,  0,
                        0, np.pi,  0]),

    # Place front feet on either side of the box CHANGE
    "q_hands_on_box_elbows_bent_under": np.array([np.pi/4, 3 * np.pi/2,  -np.pi/12,
                                                -np.pi/4, 3 * np.pi/2,  -np.pi/12,
                                                0, -np.pi/2,  np.pi,
                                                0, -np.pi/2,  np.pi]),

    # Place front feet on either side of the box CHANGE
    "q_hands_on_box_elbows_bent_sideways": np.array([(1/2) * np.pi, -1/2 * np.pi,  -(1/12) * np.pi, 
                                                    -(1/2) * np.pi, -1/2 * np.pi,  -(1/12) * np.pi,
                                                    0, -np.pi/2,  np.pi,
                                                    0, -np.pi/2,  np.pi]),


    # Final Position of Box in the air CHANGE
    "q_hold_box_up": np.array([ 1.82599084e-01,  3.57812781e+00, -1.19878120e+00,
                                -1.82599084e-01, 3.57812781e+00, -1.19878120e+00,
                                -2.48085718e-04, -1.57092560e+00,3.14168012e+00,
                                2.48085718e-04, -1.57092560e+00,  3.14168012e+00]),


    "q_box_on_back": np.array([ 0, np.pi/2,  np.pi, 
                                0, np.pi/2,  np.pi,
                                0.32631274, -1.4666653,  -1.45947616,
                                -0.32631274,-1.4666653,  -1.45947616])
}

grab_box_width = 0.1055 # y position of where the EEs grab onto the box
grab_box_height = 0.15 # z position of where the EES grab  onto the box

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

    "arms_up": {'pos': {"left": [0.1946    , 0.14795   , 0.34],
                        "right": [ 0.1946    , -0.14795   ,  0.34]}, 
                        'rgba': [0.5, 0.2, 0.3, 0.5],
                        'q_ref': robot_poses["q_arms_up_halfway"]
                },

#Get to box contact points
    "via_point1_get2box":{'pos': {"left": [0.3, grab_box_width+0.015, 0.3],
                                "right": [ 0.3, -(grab_box_width+0.015),  0.3]},
                                'rgba': [1, 0, 0, 0.5],
                                'q_ref': q_ref
                },

    "via_point2_get2box":{'pos': {"left": [0.4, grab_box_width+0.015, 0.25], # [0.4, 0.05, 0.25] push box
                        "right": [0.4, -(grab_box_width+0.015), 0.25]}, # [0.4, -0.05, 0.25] push box
                        'rgba': [1, 0, 0, 0.5],
                        'q_ref': q_ref
    },

    "box_high": {'pos': {"left": [0.45, grab_box_width, grab_box_height],
                "right": [ 0.45, -grab_box_width,  grab_box_height]},
                'rgba': [0, 1, 0, 0.5],
                'q_ref': q_ref
    },

#Lift the box up
    "via_point1_lift_box": {'pos': {"left": [0.4, grab_box_width, grab_box_height+0.06],
                        "right": [ 0.4, -grab_box_width,  grab_box_height+0.06]},
                        'rgba': [0, 1, 0, 0.5],
                        'q_ref': q_ref
    },

    "via_point2_lift_box": {'pos': {"left": [0.3, grab_box_width, grab_box_height+0.11],
                        "right": [ 0.3, -grab_box_width,  grab_box_height+0.11]},
                        'rgba': [0, 1, 0, 0.5],
                        'q_ref': q_ref
    },

    "box_up": {'pos': {"left": [0.15, grab_box_width, 0.3],
                "right": [0.15, -grab_box_width,  0.3]},
                'rgba': [0, 1, 0, 0.5],
                'q_ref': q_ref
                },

#Bring box back down
    "box_on_back_halfway":{'pos':{"left": [0.05, grab_box_width, 0.033 + 0.25],
                "right": [0.05, -grab_box_width,  0.033 + 0.25]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses["q_box_on_back"]
                },

    "box_on_back":{'pos':{"left": [0, grab_box_width, 0.033 + 0.15],
                "right": [0, -grab_box_width,  0.033 + 0.15]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses["q_box_on_back"]
                },

#Go back init position
    'back2init_viapoint_1':{'pos':{"left": [0.1946*3/5, grab_box_width+0.03, 0.033 + 0.15/2 + 0.03],
                "right": [0.1946*3/5, -(grab_box_width+0.03),  0.033 + 0.15/2 + 0.03]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses["q_box_on_back"]
                },
}

key_points_hind_legs = {

#Lift arms up
    "initial_pos": {'pos': {"left": [-0.1946    , 0.14795   , 0.03],
                        "right": [ -0.1946    , -0.14795   ,  0.03]}, 
                        'rgba': [0.5, 0.2, 0.3, 0.5],
                        'q_ref': robot_poses_hind_legs["q_init"]
                },
    
    "arms_up_halfway": {'pos': {"left": [-0.1946    , 0.14795   , 0.2],
                        "right": [ -0.1946    , -0.14795   ,  0.2]}, 
                        'rgba': [0.5, 0.2, 0.3, 0.5],
                        'q_ref': robot_poses_hind_legs["q_arms_up_halfway"]
                },

    "arms_up": {'pos': {"left": [-0.1946    , 0.14795   , 0.34],
                        "right": [ -0.1946    , -0.14795   ,  0.34]}, 
                        'rgba': [0.5, 0.2, 0.3, 0.5],
                        'q_ref': robot_poses_hind_legs["q_arms_up_halfway"]
                },

#Get to box contact points
    "via_point1_get2box":{'pos': {"left": [-0.3, grab_box_width+0.015, 0.3],
                                "right": [ -0.3, -(grab_box_width+0.015),  0.3]},
                                'rgba': [1, 0, 0, 0.5],
                                'q_ref': q_ref_hind_legs
                },

    "via_point2_get2box":{'pos': {"left": [-0.4, grab_box_width+0.015, grab_box_height+0.1], # [0.4, 0.05, 0.25] push box
                        "right": [-0.4, -(grab_box_width+0.015), grab_box_height+0.1]}, # [0.4, -0.05, 0.25] push box
                        'rgba': [1, 0, 0, 0.5],
                        'q_ref': q_ref_hind_legs
    },

    "box_high": {'pos': {"left": [-0.45, grab_box_width, grab_box_height],
                "right": [ -0.45, -grab_box_width,  grab_box_height]},
                'rgba': [0, 1, 0, 0.5],
                'q_ref': q_ref_hind_legs
    },

#Lift the box up
    "via_point1_lift_box": {'pos': {"left": [-0.4, grab_box_width, grab_box_height+0.06],
                        "right": [ -0.4, -grab_box_width,  grab_box_height+0.06]},
                        'rgba': [0, 1, 0, 0.5],
                        'q_ref': q_ref_hind_legs
    },

    "via_point2_lift_box": {'pos': {"left": [-0.3, grab_box_width, grab_box_height+0.11],
                        "right": [ -0.3, -grab_box_width,  grab_box_height+0.11]},
                        'rgba': [0, 1, 0, 0.5],
                        'q_ref': q_ref_hind_legs
    },

    "box_up": {'pos': {"left": [-0.15, grab_box_width, 0.3],
                "right": [-0.15, -grab_box_width,  0.3]},
                'rgba': [0, 1, 0, 0.5],
                'q_ref': q_ref_hind_legs
                },

#Bring box back down
    "box_on_back_halfway":{'pos':{"left": [-0.05, grab_box_width, 0.033 + 0.25],
                "right": [-0.05, -grab_box_width,  0.033 + 0.25]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses_hind_legs["q_box_on_back"]
                },

    "box_on_back":{'pos':{"left": [-0, grab_box_width, 0.033 + 0.15],
                "right": [-0, -grab_box_width,  0.033 + 0.15]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses_hind_legs["q_box_on_back"]
                },

#Go back init position
    'back2init_viapoint_1':{'pos':{"left": [-0.1946*3/5, grab_box_width+0.03, 0.033 + 0.15/2 + 0.03],
                "right": [-0.1946*3/5, -(grab_box_width+0.03),  0.033 + 0.15/2 + 0.03]},
                'rgba': [0, 0, 1, 0.5],
                'q_ref': robot_poses_hind_legs["q_box_on_back"]
                },
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

    # "back2init": [
    # key_points["box_on_back"],
    # key_points['back2init_viapoint_1'],
    # key_points["initial_pos"]
    # ],

}

key_movements_hind_legs = {

    "lift_arms": [
    key_points_hind_legs["initial_pos"],
    key_points_hind_legs["arms_up_halfway"],
    key_points_hind_legs["arms_up"]
    ],

    "get2box": [
    key_points_hind_legs["arms_up"],
    key_points_hind_legs["via_point1_get2box"],
    key_points_hind_legs["via_point2_get2box"],
    key_points_hind_legs["box_high"]
    ],

    "lift_box": [
    key_points_hind_legs["box_high"],
    key_points_hind_legs["via_point1_lift_box"],
    key_points_hind_legs["via_point2_lift_box"],
    key_points_hind_legs["box_up"]
    ],

    "box2back": [
    key_points_hind_legs["box_up"],
    key_points_hind_legs['box_on_back_halfway'],
    key_points_hind_legs["box_on_back"]
    ],

    # "back2init": [
    # key_points["box_on_back"],
    # key_points['back2init_viapoint_1'],
    # key_points["initial_pos"]
    # ],

}