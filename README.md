# Multinavigation with SLAM

## Project Description

Este proyecto implementa un sistema multi-SLAM para coordinar varios robots TurtleBot3 en un entorno simulado. Utiliza algoritmos de SLAM (Simultaneous Localization and Mapping) para fusionar mapas generados por cada robot y crear una representación global del entorno.

Está desarrollado en ROS2 y simulado en Gazebo.

## Workspace Setup

### Prerequisites
- Ubuntu 22.04
- [**ROS 2 Humble**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) installed.
- [**Gazebo 11**](https://installati.one/install-gazebo-ubuntu-22-04/) installed (I have Gazebo 11.10.2).
- TurtleBot3 dependencies:
  ```bash
  sudo apt install ros-humble-turtlebot3-msgs ros-humble-turtlebot3-gazebo ros-humble-cartographer
  ```

### Repository Cloning

```bash
# Create the workspace and navigate to the src directory
mkdir -p ~/multirobot_ws/src
cd ~/multirobot_ws/src

# Clone the repository from GitHub
git clone https://github.com/PachiElCreador/Custom-MultiSLAM-ROS2 -b main

cd multirobot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -r -y
```
### Build the workspace

```bash
cd ~/multirobot_ws
colcon build --symlink-install
```

### Configure bashrc
#### Edit the bashrc file to add sourcing and environment variables
```bash
gedit ~/.bashrc
```

#### After gedit opens, paste the following lines at the end of the document:
```bash
# Configure ROS 2 Humble
source /opt/ros/humble/setup.bash

# Configure Gazebo
. /usr/share/gazebo/setup.sh

# Configure the TurtleBot3 workspace
source ~/multirobot_ws/install/setup.bash

# Configure TurtleBot3-specific variables
export ROS_DOMAIN_ID=30 # TURTLEBOT3
export TURTLEBOT3_MODEL=burger
```
#### After saving, run the following command to apply the changes:
```bash
source ~/.bashrc
```
This will ensure that the workspace, Gazebo, and TurtleBot3 configurations are loaded in every new terminal session.

## Key Files
```plaintext
/home/user/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
```

### 1. **`multi_tb.launch.py`**
This file launches the simulation in Gazebo and spawns multiple TurtleBot3 robots.

**Purpose and Configuration:**
- Implements namespaces to separate robots.
- Uses `robot_state_publisher` and `spawn_entity.py` to initialize each robot as individual nodes.

### 2. **`slam_cart.launch.py`**
A modified file based on Cartographer to attempt SLAM with a specific namespace.

**Issues Encountered:**
- `imu_link` was not correctly registered in `tf`, causing transformation errors.
- The `map` frame was not generated correctly.

### 3. **`occupancy_grid.launch.py`**
Node that generates a map based on Cartographer data.

### 4. **`turtlebot3_gazebo`**
This directory contains the original and modified files for the TurtleBot3 simulation.

```plaintext
home/user/turtlebot3_ws/src/
|- turtlebot3_simulations/
   |- turtlebot3_gazebo/
      |- launch/
         |- multi_tb.launch.py
         |- slam_cart.launch.py
         |- occupancy_grid.launch.py
      |- urdf/
         |- turtlebot3_burger.urdf
```

## Results Achieved

### 1. Multi-Robot Simulation
- **Status:** Completed.
- **Details:** Successfully spawned two and more robots (`tb1` and `tb2`) in Gazebo with independent namespace configurations.
- **Command:**
  ```bash
  ros2 launch turtlebot3_gazebo multi_tb.launch.py
  ```

### 2. SLAM Integration with Cartographer
- **Status:** Partial.
- **Details:** While Cartographer was launched using `slam_cart.launch.py`, a map could not be displayed in `rviz` due to transformation issues (`imu_link`).

### 3. Tested Commands
- **Move robots:**
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb1/cmd_vel
  ```
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb2/cmd_vel
  ```
- **Autonomous driving:**
  ```bash
  ros2 run turtlebot3_gazebo turtlebot3_drive --ros-args -r /cmd_vel:=/tb1/cmd_vel
  ```

- **Verify transformations:**
  ```bash
  ros2 run tf2_tools view_frames
  ```

## Next Steps 
### Prioritized

#### 1. Resolve issues with `imu_link` in Cartographer
- **Difficulty:** Medium (1-4 weeks)  
- **Relevance:** High

#### 2. Validate `tf` transformations between `base_link`, `odom`, and `map`
- **Difficulty:** Medium  
- **Relevance:** High 

#### 3. Complete mapping in `rviz` and integrate navigation
- **Difficulty:** Medium 
- **Relevance:** High 

#### 4. Implement MPC together with navigation
- **Difficulty:** Probably Medium
- **Relevance:** High 

#### 5. Move from simulation to physical implementation with 2+ TurtleBots
- **Difficulty:** Low-Medium  
- **Relevance:** Medium-High

---

### Optional Tasks

#### 6. Integrate all necessary components into a single launch file
- **Difficulty:** Low-Medium
- **Relevance:** Medium (would simplify operations and enhance project reproducibility)  

#### 7. Port the project to C++ for scalability
- **Difficulty:** High
- **Relevance:** Low (not critical for immediate success but useful for larger projects)  


## Contribution
For any questions or suggestions you can contact oscarlc10@outlook.com

## References
- [TurtleBot3 Official Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
- [Cartographer ROS Integration](https://google-cartographer-ros.readthedocs.io/en/latest/)
- [ROS 2 Cartographer](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html)
