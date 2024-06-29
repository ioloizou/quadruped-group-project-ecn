
## Description

This fork implements the concepts presented in the research paper "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control" within the Quad-SDK framework. This project was developed as part of the Group Project in the Master's program in Advanced Robotics at Centrale Nantes.

## Key Features

- Implementation of the paper's convex MPC approach for dynamic locomotion control of the MIT Cheetah 3 robot.
- Integration of the control algorithm into the Quad-SDK environment.

## Paper

The paper is on the following link [Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control](https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf).

## Default Quad-SDK installation
Refer to the [Quad-SDK Wiki](https://github.com/robomechanics/quad-sdk/wiki/1.-Getting-Started-with-Quad-SDK) for installation, dependency, and unit testing information. You need to build from the noetic branch of Quad-SDK and it requires ROS Noetic on Ubuntu 20.04. All other dependencies are installed with the included setup script.

## Current fork dependecies

### Installation Instructions for OSQP and OSQP-Eigen

To run the convex MPC, you need to install the OSQP and OSQP-Eigen solvers. We recommend building from source for the most reliable installation.

### 1. Installing OSQP

* **Instructions:** Follow the official guide: https://osqp.org/docs/get_started/sources.htm
* **CMake Issue:** If you encounter problems with the CMake version, do the following:
    1. Download the latest bash script from the CMake website: https://cmake.org/download/
    2. Copy it to `/opt/`
    3. Make the script executable: `chmod +x /opt/cmake-3.*your_version*.sh`
    4. Run: `sudo bash /opt/cmake-3.*your_version*.sh` (You can change the installation directory to `/opt/` if you prefer)
    5. Create a symbolic link: `sudo ln -s /opt/cmake-3.*your_version*/bin/* /usr/local/bin`
    6. Verify the installation: `cmake --version`
* **Additional Note:** If you want to install OSQP in `/usr/local`, run the following before `make`:  `cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..`

### 2. Installing OSQP-Eigen

* **Instructions:** Follow the guide on GitHub: https://github.com/robotology/osqp-eigen
* **Additional Commands:** If you want to install in `/usr/local`, run:
    * `cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local/ ../` 
    * `export OsqpEigen_DIR=/usr/local/` (Add this to your `.bashrc` file)
    * `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib` (Add this to your `.bashrc` file)

**Important:** Make sure to adjust file paths and version numbers in commands as needed for your specific setup.

## Usage

Launch the simulation with:

```
roslaunch quad_utils quad_gazebo.launch
```

Stand the robot with:
```
rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
```
Run the stack with twist input:
```
roslaunch quad_utils quad_plan.launch reference:=twist logging:=true
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_1/cmd_vel
```
Run the stack with global planner:
```
roslaunch quad_utils quad_plan.launch reference:=gbpl logging:=true
```
Refer to the [Wiki](https://github.com/robomechanics/quad-sdk/wiki/2.-Using-the-Software) for more information on alternate usage.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).


[paper]: https://www.andrew.cmu.edu/user/amj1/papers/Quad_SDK_ICRA_Abstract.pdf
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
