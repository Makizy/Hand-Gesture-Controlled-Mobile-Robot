# Hand-Gesture_Controlled-Mobile-Robot
Control a mobile robot on a 2D map using hand gestures with the MediaPipe library, replacing traditional WASD keyboard inputs.

## Watch Our Demo



https://github.com/Makizy/Hand-Gesture-Controlled-Mobile-Robot/assets/53753128/9b24b3a9-66a4-46f5-9864-00ef70c78a68



## Project Setup and Running Guide

### Prerequisites
- Ubuntu 20.04
- ROS Noetic
- MediaPipe
- Stage ROS

### Installation

#### 1. Setting Up ROS Noetic
Follow the official ROS installation [instructions](http://wiki.ros.org/noetic/Installation/Ubuntu) for Ubuntu 20.04. Make sure to configure your environment by sourcing the ROS setup script:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2. Creating a ROS Workspace
Create a new ROS workspace to host the packages:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

#### 3. Cloning and Building the Project
Clone this repository into the `src` directory of your workspace:
```bash
cd ~/catkin_ws/src
git clone <repository-url> hand_gesture_control
cd ..
catkin_make
source devel/setup.bash
```

### Running the Project
To run the project, open three terminals and execute the following commands in sequence:

#### Terminal 1: Start ROS Core
Start the ROS master node:
```bash
roscore
```

#### Terminal 2: Launch the Stage ROS
In a new terminal, launch Stage ROS with a predefined world for simulation:
```bash
rosrun stage_ros stageros $(rospack find stage_ros)/world/willow-erratic.world
```
This command opens a 2D simulation world where the robot will navigate.

#### Terminal 3: Launch Hand Gesture Control
Finally, in another new terminal, launch the hand gesture control nodes:
```bash
roslaunch hand_gesture_control hand_gesture_control_launch.launch
```
This command starts the nodes necessary for hand gesture recognition and robot control.

### Usage
After launching all components:
- **Stage ROS** window displays the robot in a 2D environment.
- **MediaPipe** node processes hand gestures and translates them into robot movement commands.

### Contributions
We welcome contributions! Please read our contributing guidelines to learn about our review process, coding conventions, and more. Contributions can be made via pull requests to the repository.
