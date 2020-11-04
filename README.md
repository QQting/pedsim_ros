# Pedestrian Simulator

### Prerequisite

ADLINK NeuronBot2 - https://github.com/Adlink-ROS/neuronbot2

### Installation

```sh
mkdir ~/pedsim_ros2_ws/src -p
cd ~/pedsim_ros2_ws/src 
git clone https://github.com/QQting/pedsim_ros.git
cd pedsim_ros
git submodule update --init --recursive
cd ~/pedsim_ros2_ws
colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'
```

### Sample usage
```sh
ros2 launch pedsim_gazebo_plugin house_demo_launch.py
ros2 launch pedsim_simulator house_demo_launch.py
```

### Licence
The core `libpedsim` is licensed under LGPL. The ROS integration and extensions are licensed under BSD.

