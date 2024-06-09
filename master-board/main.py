"""
Code inspired from: https://github.com/open-dynamic-robot-initiative/master-board/tree/master/sdk/master_board_sdk

This script reads the predefined movement commands from a 'commands.json' file and sends these commands to the motors of the SOLO12 quadruped robot. 
By executing the commands in sequence, the robot performs the specified movements, allowing for a complete test of the trajectory planning and motor control
"""
# coding: utf8

import argparse
import math
import os
import sys
from datetime import datetime
import json
import numpy
import matplotlib.pyplot as plt
import numpy as np

sys.path.append("/home/solo/.local/lib/python3.10/site-packages")

import libmaster_board_sdk_pywrap as mbs


def execute_motion(name_interface):

    N_SLAVES = 6  #  Maximum number of controled drivers
    N_SLAVES_CONTROLED = 6  # Current number of controled drivers

    cpt = 0  # Iteration counter
    dt = 0.001  #  Time step
    t = 0  # Current time
    kp = 5.0  #  Proportional gain
    kd = 0.1 #0.5 * math.sqrt(2*kp)   # Derivative gain
    iq_sat = 2  # Maximum amperage (A)
    init_pos = [0.0 for i in range(N_SLAVES * 2)]  # List that will store the initial position of motors
    state = 0  # State of the system (ready (1) or not (0))
    saturation_count = 0
    saturation_margin = 0.7
    p_ref_array = []
    v_ref_array = []
    cur_array = []
    t_array = []
    # contrary to c++, in python it is interesting to build arrays
    # with connected motors indexes so we can simply go through them in main loop
    motors_spi_connected_indexes = [] # indexes of the motors on each connected slaves

    # Load trajectory
    with open("./commands.json") as f:
        trajectory = json.load(f)

    joint_positions = numpy.asarray(trajectory["joint_positions"])[:,-6:]
    joint_positions -= joint_positions[0]
    joint_positions = joint_positions * 9
    joint_velocities = numpy.asarray(trajectory["joint_velocities"])[:,-6:]
    joint_velocities = joint_velocities*9
    joint_positions[:,1:4] = -joint_positions[:,1:4]
    joint_velocities[:, 1:4] = -joint_velocities[:,1:4]

    # joint_positions2 = numpy.asarray([joint_positions[0] for q in joint_positions])
    # joint_velocities = numpy.zeros_like(joint_velocities)
    # joint_positions = joint_positions2

    os.nice(-20)  #  Set the process to highest priority (from -20 highest to +20 lowest)
    robot_if = mbs.MasterBoardInterface(name_interface)
    robot_if.Init()  # Initialization of the interface between the computer and the master board
    for i in range(N_SLAVES_CONTROLED):  #  We enable each controler driver and its two associated motors
        robot_if.GetDriver(i).motor1.SetCurrentReference(0)
        robot_if.GetDriver(i).motor2.SetCurrentReference(0)
        robot_if.GetDriver(i).motor1.Enable()
        robot_if.GetDriver(i).motor2.Enable()
        robot_if.GetDriver(i).EnablePositionRolloverError()
        robot_if.GetDriver(i).SetTimeout(5)
        robot_if.GetDriver(i).Enable()

    last = datetime.now()

    while (not robot_if.IsTimeout() and not robot_if.IsAckMsgReceived()):
        if ((datetime.now() - last).total_seconds() > dt):
            last = datetime.now()
            robot_if.SendInit()

    if robot_if.IsTimeout():
        print("Timeout while waiting for ack.")
    else:

        # fill the connected motors indexes array
        for i in range(N_SLAVES_CONTROLED):
            if robot_if.GetDriver(i).IsConnected():
                # if slave i is connected then motors 2i and 2i+1 are potentially connected
                motors_spi_connected_indexes.append(2 * i)
                motors_spi_connected_indexes.append(2 * i + 1)
    
    start = datetime.now()

    motors_spi_connected_indexes = [ 6, 9, 8, 7, 11, 10]

    for i in motors_spi_connected_indexes:
        print(robot_if.GetMotor(i).GetPosition())

    while ((not robot_if.IsTimeout())
           and cpt < joint_positions.shape[0]-1):  

        if ((datetime.now() - last).total_seconds() > dt):
            last = datetime.now()
            cpt += 1
            t += dt
            robot_if.ParseSensorData()  # Read sensor data sent by the masterboard

            if (state == 0):  #  If the system is not ready
                state = 1

                # for all motors on a connected slave
                for i in motors_spi_connected_indexes:  # Check if all motors are enabled and ready
                    if not (robot_if.GetMotor(i).IsEnabled() and robot_if.GetMotor(i).IsReady()):
                        state = 0
                    init_pos[i] = robot_if.GetMotor(i).GetPosition()
                    t = 0

            else:  # If the system is ready

                # for all motors on a connected slave
                for idx,i in enumerate(motors_spi_connected_indexes):

                    if i % 2 == 0 and robot_if.GetDriver(i // 2).GetErrorCode() == 0xf:
                        #print("Transaction with SPI{} failed".format(i // 2))
                        continue #user should decide what to do in that case, here we ignore that motor

                    if robot_if.GetMotor(i).IsEnabled():
                        ref = joint_positions[cpt,idx]
                        v_ref = joint_velocities[cpt,idx]

                        p_err = ref - robot_if.GetMotor(i).GetPosition()  # Position error
                        v_err = v_ref - robot_if.GetMotor(i).GetVelocity()  # Velocity error
                        cur = kp * p_err + kd * v_err  #  Output of the PD controler (amperage)

                        if i == 6:
                            p_ref_array.append(ref)
                            v_ref_array.append(v_ref)
                            cur_array.append(cur)
                        if (cur > iq_sat):  #  Check saturation
                            if abs(cur - iq_sat) > saturation_margin:
                                saturation_count += 1
                            cur = iq_sat
                        if (cur < -iq_sat):
                            if abs(cur + iq_sat) > saturation_margin:
                                saturation_count += 1
                            cur = -iq_sat
                        robot_if.GetMotor(i).SetCurrentReference(cur)  # Set reference currents

            #if ((cpt % 100) == 0):  # Display state of the system once every 100 iterations of the main loop
                
            #    print(chr(27) + "[2J")
            #    # To read IMU data in Python use robot_if.imu_data_accelerometer(i), robot_if.imu_data_gyroscope(i)
            #    # or robot_if.imu_data_attitude(i) with i = 0, 1 or 2
            #    robot_if.PrintIMU()
            #    robot_if.PrintADC()
            #    robot_if.PrintMotors()
            #    robot_if.PrintMotorDrivers()
            #    robot_if.PrintStats()
            #    sys.stdout.flush()  # for Python 2, use print( .... , flush=True) for Python 3

            robot_if.SendCommand()  # Send the reference currents to the master board

    robot_if.Stop()  # Shut down the interface between the computer and the master board

    if robot_if.IsTimeout():
        print("Masterboard timeout detected.")
        print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
    print(f'number of saturations with a margin superieror to {saturation_margin}: {saturation_count}')
    print("-- End of example script --")

    fig, axs = plt.subplots(3, sharex=True, sharey=True)
    fig.suptitle('ref, v_ref, cur')
    axs[0].plot(p_ref_array)
    axs[1].plot(v_ref_array, 'o')
    axs[2].plot(cur_array, '+')
    axs[2].set_xlabel('Time (s)')
    axs[0].set_xlabel('target position')
    axs[1].set_xlabel('taregt velocity')
    axs[2].set_xlabel('current (A)')

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Example masterboard use in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    execute_motion(parser.parse_args().interface)


if __name__ == "__main__":
    main()
