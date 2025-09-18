# UR5 + Robotiq (Gazebo + MoveIt)

This repository contains a portable Gazebo simulation of a **UR5** equipped with a **Robotiq 2F gripper**, plus a separate MoveIt config for motion planning.

- Simulation package: **`ur5_tsukii`**  
- MoveIt config package: **`ur5_camera_moveit_gripper`** (separate repo)

The setup below gets you from a clean Humble installation to running the robot in Gazebo with controllers and MoveIt.

---

## Prerequisites

- Ubuntu **22.04**
- ROS 2 **Humble** installed (desktop or desktop-full)
- A working OpenGL/X11 setup for GUI apps (Gazebo, RViz)

Install required system/ROS packages:

```bash
sudo apt update && sudo apt install -y \
  git build-essential cmake python3-pip nano curl wget \
  ros-dev-tools python3-colcon-common-extensions \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros gazebo libgazebo-dev \
  ros-humble-moveit ros-humble-moveit-ros-visualization \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  mesa-utils libgl1-mesa-dri libxkbcommon0 \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-realtime-tools ros-humble-control-toolbox ros-humble-control-msgs
```

## Create a workspace

```bash
# create and enter a workspace
mkdir -p ~/ur_ws/src
cd ~/ur_ws

# source ROS 2 (add this to your ~/.bashrc if you haven't)
source /opt/ros/humble/setup.bash
```

## Clone required repository:

- Clone these into ~/ur_ws/src:
```bash
cd ~/ur_ws/src

# UR descriptions and Gazebo sim
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git

# Robotiq gripper (ROS 2)
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git

# MoveIt config (separate repo)
git clone https://github.com/LearnRoboticsWROS/ur5_camera_moveit_config.git

# This simulation package (the repo you're reading)
git clone https://github.com/LearnRoboticsWROS/ur5_tsukii.git
```

## Install dependencies

- From the workspace root:
```bash
cd ~/ur_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true
```
The || true prevents the command from failing the whole script if optional deps are missing.


## Build
- We skip hardware-only Robotiq drivers (not needed for simulation):
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install \
  --packages-skip robotiq_driver robotiq_hardware_tests
```

Source your overlay
```bash
source ~/ur_ws/install/setup.bash
```
Everytime you open a terminal you always need to source /opt/ros/humble/setup.bash and source ~/ur_ws/install/setup.bash
Consider to add those line to your ~/.bashrc file

## Run
- Launch Gazebo 
```bash
# with RViz
ros2 launch ur5_tsukii spawn_ur5_gripper_moveit.launch.py with_rviz:=true

# without RViz
# ros2 launch ur5_tsukii spawn_ur5_gripper_moveit.launch.py with_rviz:=false
```

If everything is OK you should see:

Gazebo world with the UR5 + Robotiq gripper spawned,

controllers loaded (joint_state_broadcaster, joint_trajectory_controller, gripper_position_controller),

MoveIt’s move_group running,

RViz showing the robot state (when with_rviz:=true).

## Notes & Throubleshooting
First launch is slow: Gazebo may freeze for a bit while loading plugins and compiling shaders. That’s normal on the first run.

Controller spawners keep “waiting for /controller_manager”: Gazebo might still be initializing. Give it a few seconds. If it persists, close everything and relaunch.

Gripper/Camera visible in Gazebo but not in RViz: make sure you launched with with_rviz:=true. The URDF is generated from xacro with package-based mesh paths, so RViz and Gazebo will both resolve meshes from the installed workspace.

Models not found: the launch sets Gazebo paths, but if you customized things, ensure GAZEBO_MODEL_PATH, GAZEBO_RESOURCE_PATH and GAZEBO_PLUGIN_PATH include your workspace. (Sourcing ~/ur_ws/install/setup.bash usually handles it.)

Rebuild after changes: if you edit xacros/URDF, re-run colcon build and re-source ~/ur_ws/install/setup.bash.

## Repository layout (high level)
- ur5_tsukii/ – Gazebo simulation package (URDF/Xacro, controllers, launch)

- ur5_camera_moveit_config/ – MoveIt configuration for the UR5 + gripper

- Universal_Robots_ROS2_Description/ – Official UR descriptions

- Universal_Robots_ROS2_Gazebo_Simulation/ – UR Gazebo integration

- ros2_robotiq_gripper/ – Robotiq 2F gripper stack for ROS 2

- IFRA_LinkAttacher/ – If you want to simulate a pick and place
