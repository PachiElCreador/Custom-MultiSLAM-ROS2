# Multinavigation with SLAM

## Project Description

Este proyecto implementa un sistema multi-SLAM para coordinar varios robots TurtleBot3 en un entorno simulado. Utiliza algoritmos de SLAM (Simultaneous Localization and Mapping) para fusionar mapas generados por cada robot y crear una representación global del entorno.

Está desarrollado en ROS2 y simulado en Gazebo.

## Workspace Setup

### Prerequisites
- Ubuntu 22.04
- [**ROS 2 Humble**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) installed.
- [**Gazebo 11**](https://installati.one/install-gazebo-ubuntu-22-04/) installed (I have Gazebo 11.10.2).
- [TurtleBot3 dependencies](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/):
  <details>
    <summary>View turtlebot3 installation commands</summary>
  
    ```bash
    # Dependent ROS2 Packages
    sudo apt install ros-humble-gazebo-*
    
    sudo apt install ros-humble-cartographer
    sudo apt install ros-humble-cartographer-ros
  
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    
    # Install TurtleBot3 Packages
    mkdir -p ~/turtlebot3_ws/src
    cd ~/turtlebot3_ws/src/
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    sudo apt install python3-colcon-common-extensions
  
    # Install tb3 simulation package
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  
    # Build the workspace
    cd ~/turtlebot3_ws
    colcon build --symlink-install
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
    source ~/.bashrc
    ```
  
  </details>

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

# Configure the multirobot_ws workspace
source ~/multirobot_ws/install/setup.bash

# Ensure turtlebot3 workspace is sourced
source ~/turtlebot3_ws/install/setup.bash

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

### **1. `gazebo_multi_robot_house.launch.py`**
This is the main launch file orchestrating the multi-robot simulation. It:
- Defines the initial positions and names for multiple robots.
- Launches the Gazebo server and client with a predefined world (`turtlebot3_house.world`).
- Spawns multiple TurtleBot3 robots with URDF models and ensures sequential initialization using event handlers.
- Optionally enables SLAM or navigation through other launch files.
- Allows launching RViz for visualization and sets up navigation-related nodes for each robot.

---

### **2. `bringup/bringup_launch.py`**
This file integrates core functionalities such as SLAM, localization, and navigation. It:
- Manages the namespaces for robots and ensures configurations are isolated.
- Dynamically includes additional launch files (`slam_launch.py`, `localization_launch.py`, and `navigation_launch.py`) based on conditions like SLAM or map server usage.
- Centralizes configuration parameters through YAML files and remaps transform topics.

---

### **3. `bringup/localization_launch.py`**
Handles the localization stack for the robots. It:
- Loads the AMCL node (`Adaptive Monte Carlo Localization`) to localize robots using a pre-existing map.
- Optionally launches the map server to provide static map data.
- Configures lifecycle management for AMCL and the map server to handle node states.

---

### **4. `bringup/navigation_launch.py`**
Sets up the navigation stack, enabling path planning and movement. It:
- Launches core navigation nodes, including `planner_server`, `controller_server`, `bt_navigator`, and others.
- Configures the lifecycle manager to handle the state of navigation nodes.
- Supports remapping topics for sensor inputs and control outputs to maintain isolation between robots.

---

### **5. `bringup/rviz_launch.py`**
Manages the launch of RViz for visualization. It:
- Loads the RViz configuration file (`nav2_default_view.rviz`).
- Dynamically sets up namespaces for visualization if multiple robots are used.
- Handles RViz shutdown gracefully to avoid unintended crashes.

---

### **6. `bringup/slam_launch.py`**
This file initializes the SLAM functionality. It:
- Launches the SLAM Toolbox (`online_sync_launch.py`) for map generation and updates.
- Optionally launches the map saver server to store generated maps.
- Includes lifecycle management to ensure the proper startup and shutdown of SLAM-related nodes.


```plaintext
home/user/multirobot_ws/src/
   |- turtlebot3_multi_robot/
      |- launch/
         |- gazebo_multi_robot_house.launch.py
         |- bringup
            |- bringup_launch.py
            |- localization_launch.py
            |- navigation_launch.py
            |- rviz_launch.py
            |- slam_launch.py
```

## Results Achieved

### 1. Multi-Robot Simulation in Gazebo
- **Status:** Completed.
- **Details:** The simulation successfully spawns the TurtleBot3 robots in the Gazebo environment as expected. The robots are positioned at their predefined coordinates, and their transformations are correctly published using the `robot_state_publisher`. Gazebo runs smoothly, and the integration with ROS 2 ensures seamless interaction between nodes.

### 2. SLAM Functionality in RViz
- **Status:** Completed, but with possible improvements.
- **Details:** The SLAM functionality in RViz works decently, showing real-time mapping and navigation data. However, there are areas for improvement in the localization accuracy of the TurtleBot3 robots, as occasional discrepancies are observed between the robots' perceived positions and their actual locations in the simulation. These discrepancies may affect the overall performance of navigation tasks.

### 3. Future Directions
- **Improve SLAM algorithm:** Focus on enhancing mapping accuracy and ensuring precise localization of the robots.
- **Collaborative Mapping:** Enable multiple robots to collaboratively share and update a global map.
- **Robot Coordination:** Implement mechanisms for robots to account for the presence and movements of other robots in the environment, fostering better coordination and efficiency in navigation tasks.


## Contribution
For any questions or suggestions you can contact oscarlc10@outlook.com

## References
- [TurtleBot3 Official Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
- [Cartographer ROS Integration](https://google-cartographer-ros.readthedocs.io/en/latest/)
- [ROS 2 Cartographer](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html)
