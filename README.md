# solo12_simulation

Groundwork laid by the original project by `rkourdis/solo12_mujoco`: https://github.com/rkourdis/solo12_mujoco.git.

## Instructions
1. Install requirements: `pip3 install -r requirements.txt`
1. Generate the Solo-12 model: `python3 ./generate_model.py`
    - This will write an `.xml` file with the geometry of the four legs imported and correctly oriented

## Using MacOS
>⚠️ On MacOS, `launch_passive` requires that the user script is executed via a special `mjpython` launcher. The `mjpython` command is installed as part of the `mujoco` package, and can be used as a drop-in replacement for the usual python command and supports an identical set of command line flags and arguments. For example, a script can be executed via `mjpython my_script.py`, and an IPython shell can be launched via `mjpython -m IPython`.
