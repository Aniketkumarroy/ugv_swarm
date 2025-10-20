# UGV Swarm
this is a ros simulation of/swarm of differential drive robot
- [x] configurable params to launch multiple number of robots
- [ ] modularize robot urdf and integration of different modules as plug and play
- [ ] parameterized enabling and disabling of different sensor suits
---

## Build
clone the repository into your ros2 ws
```bash
cd <your_ros2_ws>
mkdir src
cd src
git clone https://Aniketkumarroy/ugv_swarm.git
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
