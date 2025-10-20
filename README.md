# UGV Swarm
this is a ros simulation of/swarm of differential drive robot
- [x] configurable params to launch multiple number of robots
- [ ] modularize robot urdf and integration of different modules as plug and play
- [ ] parameterized enabling and disabling of different sensor suits
---
## Prerequisites
this package requires ros foxy and gazebo classic.
for foxy you can first have a Ubuntu 20.04 setup and then install ros foxy following [this](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) or you can use a docker container from osrf
```bash
docker pull osrf/ros:foxy-desktop
```
for gazebo classic just install the default gazebo of foxy
```bash
sudo apt install ros-foxy-gazebo-ros
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-gazebo-ros-msgs
```
also since we will be using xacro for making robot urdf you also need to install xacro
```bash
sudo apt install ros-foxy-xacro
```

## Build
clone the repository into your ros2 ws
```bash
cd <your_ros2_ws>
mkdir src
cd src
git clone https://github.com/Aniketkumarroy/ugv_swarm.git
```
now we need to get some packages for core functionalities of navigation, slam and also some world files. use vcs + rosdep get it
```bash
cd <your_ros2_ws>
vcs import --input src/ugv_swarm/dependencies.repos src
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
although rosdep should have done everything still these are some dependencies packages which are missed by rosdep, install it
```bash
sudo apt install ros-foxy-test-msgs
sudo apt install ros-foxy-behaviortree-cpp-v3
sudo apt install libompl-dev
```
now we build it
```bash
cd <your_ros2_ws>
colcon build
```
now leave your laptop for a while as it will take time and compute

## Running
to launch only one bot in the world type
```
ros2 launch ugv_swarm launch_one_bot.launch.py
```
by default the world is empty, to load your preferred world use `world` argument
```
ros2 launch ugv_swarm launch_one_bot.launch.py world:=src/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
```
if you also want to see complete verbose output of gazebo use `verbose:=true`. if you want to disable gazebo simulation GUI and run headless use `gui:=false` 
