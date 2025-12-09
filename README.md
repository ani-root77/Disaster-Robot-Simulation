# Disaster Robot Simulation (ROS2 + Gazebo + RL)

Simulation of a differential-drive disaster response robot in a custom obstacle-filled world, built with **ROS2 Humble**, **Gazebo**, and a **Python RL environment (Gymnasium)**.

The project lets you:
- Spawn a mobile robot with LiDAR in a disaster-like maze.
- Interact with it via ROS2 topics.
- Train and evaluate simple RL agents (e.g. Q-learning–style) before moving to PPO / actor–critic.

---

## 1. Project Structure

disaster_robot_ws/
├── src/
│ └── disaster_robot_description/
│ ├── urdf/
│ │ └── disaster_robot.urdf
│ ├── worlds/
│ │ └── disaster_obstacles.world
│ ├── launch/
│ │ └── robot_gazebo.launch.py
│ ├── package.xml
│ └── CMakeLists.txt
└── install/ # created by colcon build

rl_robot_env/ # Python virtual environment & RL code
├── disaster_rl_env.py # Gymnasium-ROS2 environment
└── simple_train.py # Simple baseline “agent” to test the pipeline

- `disaster_robot_description`: ROS2 package with robot model, world, and launch files.
- `disaster_rl_env.py`: wraps ROS2 (topics, actions, rewards) into a Gym-style environment.
- `simple_train.py`: runs episodes with a very simple policy to verify everything works.

---

## 2. Prerequisites

- **OS**: Ubuntu 22.04 (native or WSL2)
- **ROS2**: Humble
- **Gazebo**: Classic (11.x) with `gazebo_ros_pkgs`
- **Python**: 3.10+ with virtualenv

Core packages (Ubuntu):

sudo apt update
sudo apt install -y
ros-humble-desktop
ros-humble-gazebo-ros-pkgs
python3-colcon-common-extensions
python3-venv python3-pip git

---

## 3. Setup

### 3.1 ROS2 Workspace

mkdir -p ~/disaster_robot_ws/src
cd ~/disaster_robot_ws
colcon build --symlink-install

Add to `~/.bashrc`:

source /opt/ros/humble/setup.bash
source ~/disaster_robot_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

Then: source ~/.bashrc

### 3.2 Python Virtual Environment & RL Dependencies

python3 -m venv ~/rl_robot_env
source ~/rl_robot_env/bin/activate
pip install --upgrade pip
pip install gymnasium numpy rclpy stable-baselines3 torch

(Adjust RL libs as needed.)

---

## 4. Running the Simulation

### 4.1 Launch Gazebo + Robot

In a terminal:
source ~/rl_robot_env/bin/activate
source /opt/ros/humble/setup.bash
source ~/disaster_robot_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch disaster_robot_description robot_gazebo.launch.py

This will:
- Start Gazebo with `disaster_obstacles.world`.
- Spawn `disaster_robot` from `disaster_robot.urdf`.
- Bring up ROS2 topics like `/scan`, `/odom`, `/cmd_vel`.

### 4.2 Inspect ROS2 Topics

In another terminal:

source /opt/ros/humble/setup.bash
source ~/disaster_robot_ws/install/setup.bash

ros2 topic list
ros2 topic echo /scan # LiDAR data
ros2 topic echo /odom # Odometry

---

## 5. RL Environment

### 5.1 `disaster_rl_env.py` (Overview)

Key ideas:
- **Observation**: latest LiDAR scan (`/scan`), e.g. 360 beams → vector of distances.
- **Action space**: discrete set of velocity commands (forward, back, left, right, stop) mapped to `/cmd_vel`.
- **Reward**:
  - Step penalty: encourage shorter paths.
  - Large positive reward when reaching the goal region.
  - Negative reward on collision (LiDAR too close to obstacle).
- **Episode termination**:
  - Goal reached.
  - Collision.
  - Max steps exceeded.

Run a quick manual rollout:

source ~/rl_robot_env/bin/activate
python ~/rl_robot_env/disaster_rl_env.py


You should see step info printed and the robot move around in Gazebo.

---

## 6. Simple Baseline “Agent”

`simple_train.py` runs multiple episodes using a naive/random policy against `DisasterRobotEnv`:

source ~/rl_robot_env/bin/activate
python ~/rl_robot_env/simple_train.py


This is used only to verify:
- Observations are received from ROS2.
- Actions correctly control the robot.
- Reward and termination flags behave as expected.

Once this works, the same interface can be plugged into PPO / actor–critic from libraries like Stable-Baselines3.

---

## 7. Extending the Project

Ideas for next steps:

- Replace the simple baseline with PPO / actor–critic.
- Improve the reward function using:
  - Distance-to-goal from odometry.
  - Collision counts or contact sensors.
- Add curriculum (vary initial pose, debris layout, or goal positions).
- Log training metrics and trajectories for analysis.

---

## 8. Troubleshooting

- **Gazebo opens with empty world**  
  Check `robot_gazebo.launch.py` points to `worlds/disaster_obstacles.world`.

- **Robot does not move**  
  Ensure you publish to `/cmd_vel` and that the diff-drive plugin is loaded in the URDF.

- **No LiDAR data**  
  Confirm LiDAR is defined in the URDF and `/scan` topic exists (`ros2 topic list`).

---

## 9. License

Add your preferred license here (e.g. MIT, BSD, Apache-2.0).

---

## 10. Acknowledgements

- ROS2 and Gazebo communities for tooling and tutorials.
- Reinforcement learning frameworks (Gymnasium, Stable-Baselines3) for agent interfaces.
