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
# Pleae source NeuronBot2 and pedsim for each terminals
source ~/neuronbot2_ros2_ws/install/local_setup.bash
source ~/pedsim_ros2_ws/install/local_setup.bash

# Terminal 1
ros2 launch pedsim_gazebo_plugin neuronbot2_world_launch.py

# Terminal 2
ros2 launch pedsim_simulator neuronbot2_demo_launch.py simulation_factor:=1.0

# Terminal 3
ros2 launch neuronbot2_nav bringup_launch.py use_sim_time:=true default_bt_xml_filename:=/home/ros/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_nav/param/follow_point.xml

# Terminal 4
rviz2 -d ~/pedsim_ros2_ws/src/pedsim_ros/pedsim_visualizer/rviz/pedsim.rviz 
```

### Licence
The core `libpedsim` is licensed under LGPL. The ROS integration and extensions are licensed under BSD.

