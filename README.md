### • robot_movement.cpp
This script implements a simple ROS 2 node to control the robot’s movement based on its odometry. Its main features are:

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
