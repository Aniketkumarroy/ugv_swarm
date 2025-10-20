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

```bash
vcs import --input src/ugv_swarm/dependencies.repos src
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

```bash
sudo apt install ros-foxy-test-msgs
sudo apt install ros-foxy-behaviortree-cpp-v3
sudo apt install libompl-dev
```
