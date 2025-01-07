# Assignment 2.2 - Robot simulation

## **Overview**
This package implements a simple ROS 2 node to control the robot’s movement based on its odometry.

## **Getting Started**
### 1. Install ROS2
In order to run the project, ROS2 must be installed.
It is also needed to run this commands:

```bash
apt-get update
apt-get upgrade (it may take a while!)
apt-get install ros-foxy-xacro ros-foxy-joint-state-publisher ros-foxy-gazebo*
```

### 2. Clone the repository

Navigate to your workspace and clone the repository into the `src` folder:

```bash
git clone <URL_of_this_repository>
```

### 3. Clone the gazebo simulation
```bash
git clone https://github.com/CarmineD8/robot_urdf.git
```

### 4. Run the gazebo simulation:

```bash
ros2 launch robot_urdf gazebo.launch.py
```

### 5. Launch the code

```bash
ros2 run assignment2_rt_part2 robot_movement
```

## How it works
### • robot_movement.cpp
Its main features are:

#### - Odometry Subscription
The node subscribes to the `/odom` topic to receive the robot's position data (x, y).

#### - Movement Logic
Based on the robot’s x-coordinate:
- If `x > 9.0`, it moves forward with a positive angular velocity.
- If `x < 1.5`, it moves forward but with a negative angular velocity (turning in the opposite direction).
- If `1.5 <= x <= 9.0`, it moves straight without turning.

#### - Velocity Publishing
The node publishes a `Twist` message to the `/cmd_vel` topic to control the robot’s linear and angular velocities based on the x and y position received from the odometry.

#### - Stopping Condition
If `y > 9`, the robot stops moving (linear and angular velocities set to zero).

This node continuously adjusts the robot's velocity based on its position and publishes the appropriate command to the robot's velocity topic.
