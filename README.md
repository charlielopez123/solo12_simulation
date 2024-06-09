# solo12_simulation

Groundwork laid by the original project by `rkourdis/solo12_mujoco`: https://github.com/rkourdis/solo12_mujoco.git.

## Introduction

This project aims to implement a robust and efficient trajectory planning and control system for the SOLO12 quadruped robot. The system is designed to handle manipulation tasks involving precise coordination of the robot's end-effectors, ensuring smooth and natural movements.

## Instructions
1. Install requirements: `pip3 install -r requirements.txt`
1. Generate the Solo-12 model: `python3 ./generate_model.py`
    - This will write an `.xml` file with the geometry of the four legs imported and correctly oriented

## Using MacOS
>⚠️ On MacOS, `launch_passive` requires that the user script is executed via a special `mjpython` launcher. The `mjpython` command is installed as part of the `mujoco` package, and can be used as a drop-in replacement for the usual python command and supports an identical set of command line flags and arguments. For example, a script can be executed via `mjpython my_script.py`, and an IPython shell can be launched via `mjpython -m IPython`.

## General Structure

- Simulation.py: defines the simulation environment used for this project, including setting up the MuJoCo physics simulator, configuring the SOLO12 quadruped robot, and implementing all the main functions necessary for trajectory planning, optimization, and control, such as the IK Solver, TOPPRA algorithm for velocity profile generation, and animation of the robot's movements.
- solo.py: defines a class `Robot` for handling robotic manipulator operations, including forward and inverse kinematics, joint control, and optimization for inverse kinematics.
- Joint_positions.py: defines key points, movements, and reference configurations for a robot performing tasks such as picking up a box.

- Codes:  various scripts designed to test different functionalities of the project, including individual components of the trajectory planning pipeline with the creation of the commands.json file and scripts to visualize the full trajectory using the TOPPRA algorithm