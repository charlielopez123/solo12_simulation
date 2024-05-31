# Testing of inverse_kinematics_adjusted() function for holding the box up position using 'q_arms_up' as a q_ref for the solver
import mujoco
import mujoco.viewer as viewer
import numpy as np
from solo import Robot
from time import sleep
import time
from Simulation import SoloSim
from Joint_positions import *

q_init = robot_poses["q_init"]
sim = SoloSim(q_init = q_init, use_hind_legs=True)

sim.all_the_points(key_points_hind_legs)

for_loop = True

if for_loop:
    for pt in key_points_hind_legs:
        print(pt) #which point are we going to
        x_des = sim.x_des(target = key_points_hind_legs[pt])
        q = sim.inverse_kinematics_adjusted(x_des, q_ref = key_points_hind_legs[pt]['q_ref'])
        #print(sim.robot.get_q()[8])
        sim.animate(q, t_max=0.5)

else:
    x_des = sim.x_des(target = key_points_hind_legs["arms_up_halfway"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)#robot_poses_hind_legs["q_arms_up_halfway"]
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["arms_up"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)#robot_poses_hind_legs["q_arms_up_halfway"]
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["via_point1_get2box"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["via_point2_get2box"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["box_high"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["via_point1_lift_box"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["via_point2_lift_box"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["box_up"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)#robot_poses_hind_legs["q_hold_box_up"]
    sim.animate(q)

    x_des = sim.x_des(target = key_points_hind_legs["box_on_back"])
    q = sim.inverse_kinematics_adjusted(x_des, q_ref = q_ref_hind_legs)
    sim.animate(q)