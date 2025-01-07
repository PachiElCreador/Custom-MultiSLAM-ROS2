# Useful Commands and Resources

This document contains commands, configurations, and notes relevant to managing the multi-robot simulation and related tools.

---

## Access and Configuration

### Open the file where the map is saved
Also useful to access the shared directory of ros2 humble
```bash
sudo nautilus /opt/ros/humble/share/turtlebot3_navigation2/map
```

## Simulation multirobot_ws

### Launch Multi-Robot House Simulation
```bash
cd multirobot_ws/
colcon build --symlink-install
source ./install/setup.bash
ros2 launch turtlebot3_multi_robot gazebo_multi_robot_house.launch.py #enable_drive:=True enable_rviz:=False
```

#### Teleoperation Mode
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb1/cmd_vel
```
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb2/cmd_vel
```

#### Start SLAM for `tb1` and `tb2`
```bash
ros2 launch slam_toolbox online_sync_launch.py namespace:=/tb1 use_sim_time:=true
```
```bash
ros2 launch slam_toolbox online_sync_launch.py namespace:=/tb2 use_sim_time:=true
```

---

## Launch simulation of Robotis example
```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
```
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

#### Run Teleoperation Mode:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

#### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## Debugging and Process Management

### Terminate Residual Processes
```bash
pkill -f gazebo
pkill -f rviz
pkill -f ros2
```

### List Active Processes
```bash
ps aux | grep gazebo
ps aux | grep ros2
ps aux | grep rviz
```

### List Active Nodes
```bash
ros2 node list
```

### List Available Topics
```bash
ros2 topic list
```

### Generate a PDF of the Current TF Tree
```bash
ros2 run tf2_tools view_frames
```


