# AUTONOMOUS VEHICLE: CONTROL AND BEHAVIOUR

<p align="center"><b>AutoCarROS has migrated to ROS 2 Foxy Fitzroy</b></p>

<div align="center">
    <img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/pictures/ngeeann_av_ultrawide.png?raw=true" />
</div>

## Abstract

This project contains the ROS 2 variant of the [AutoCarROS](https://github.com/winstxnhdw/AutoCarROS) repository. It is a template for the development of a robust non-holonomic autonomous vehicle platform in a simulated environment using ROS 2 and Gazebo 11.
> The following GIF demonstrates a simulation built on top of AutoCarROS 2.

<div align="center">
    <img src="resources/reactive_path_planning.gif" />
</div>

## Installation

Create a workspace

```bash
$ mkdir -p PATH/TO/WORKSPACE/src
$ cd src
```

Clone the repository.

```bash
$ git clone https://github.com/winstxnhdw/AutoCarROS2.git
$ cd PATH/TO/WORKSPACE/src/AutoCarROS2
```

Install ROS 2 **and** the required dependencies.

```bash
sh ros-foxy-desktop-full-install.sh
```

If you only need to install the required dependencies, run the following. Otherwise, skip this step.

```bash
sh requirements.sh
```

Build the packages.

```bash
$ cd PATH/TO/WORKSPACE/
$ colcon build
```

Append the workspace to .bashrc.

```bash
$ echo "source PATH/TO/WORKSPACE/install/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

## Usage

When using this project for the first time, it is necessary that the user builds the packages before attempting to run the launch files.

```bash
# Change directory to your desired workspace
cd PATH/TO/WORKSPACE/

# Build packages
$ colcon build
```

There are two launch files the user can use. More details in the [Launch Files](#Launch-Files) section.

```bash
# Launch the default launch file
$ ros2 launch launches default_launch.py

# OR

# Launch the interactive launch file
$ ros2 launch launches click_launch.py

# Pure Pursuit race stack (build autocar_nav_pure_pursuit first)
$ colcon build --packages-select autocar_nav autocar_nav_pure_pursuit launches --symlink-install
$ source install/setup.bash
$ ros2 launch launches race_pure_pursuit_launch.py

# MPC race stack (lap_timer is in autocar_nav)
$ colcon build --packages-select autocar_nav autocar_nav_mpc launches --symlink-install
$ source install/setup.bash
$ ros2 launch launches race_mpc_launch.py
# After a lap: compare vs baseline
$ python3 scripts/compare_lap_times.py --detail
```

## Launch Files

|Launch File|Purpose|
|-----------|-------|
|`default_launch.py`|Complete pipeline with preset waypoints|
|`click_launch.py`|Interactive pipeline for testing and fun|
|`race_pure_pursuit_launch.py`|Race circuit with Pure Pursuit tracker (higher speed)|
|`race_mpc_launch.py`|Race circuit with Model Predictive Control tracker|

## Packages

|Package|Purpose|
|-----------|-------|
|`launches`|Contains the main launch files for quick launching|
|`autocar_description`|Contains the model's URDF and RViz configuration files|
|`autocar_gazebo`|Contains the world files and model's SDF|
|`autocar_map`|Contains the Bayesian Occupancy Filter stack|
|`autocar_msgs`|Contains all custom messages used throughout every package|
|`autocar_nav`|Contains the navigation stack (Stanley tracker)|
|`autocar_nav_pure_pursuit`|Pure Pursuit navigation stack (faster, curvature-aware speed)|
|`autocar_nav_mpc`|Model Predictive Control navigation stack (see [`docs/MPC_REPORT.md`](../../docs/MPC_REPORT.md))|

## Troubleshoot

There are occasions where `colcon build` does not properly rebuild the 'build' and 'install' folders, especially when one has made changes to the `CMakeLists.txt`. In the following, a simple quick fix can be performed.

```bash
# Remove build and install files
$ cd PATH/TO/WORKSPACE/
$ rm -rf build install
```

## Renders

<p align="center"><b>"Because the layman doesn't care unless it looks cool."</b></p>

<div align="center">
    <img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/renders.gif?raw=true" />
    <img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/1.gif?raw=true" />
    <img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/2.gif?raw=true" />
    <img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/3.gif?raw=true" />
    <img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/4.gif?raw=true" />
</div>
