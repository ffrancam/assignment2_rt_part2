#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
double current_x = 0.0;
double current_y = 0.0;

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;

    geometry_msgs::msg::Twist cmd_vel_msg;
 
	if (current_y > 9) {
		cmd_vel_msg.linear.x = 0.0; 
        cmd_vel_msg.angular.z = 0.0; 
	}
	
    // Inver velocities
    if (current_x > 9.0) {
        cmd_vel_msg.linear.x = 1.0; 
        cmd_vel_msg.angular.z = 3.0; 
    }
    else if (current_x < 1.5) {
        cmd_vel_msg.linear.x = 1.0;    
        cmd_vel_msg.angular.z = -3.0; 
    }
    else 
    {
        cmd_vel_msg.linear.x = 1.0; 
        cmd_vel_msg.angular.z = 0.0; 
    }

    pub->publish(cmd_vel_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Initialize the node
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_robot_node");

    // Publisher
    pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscriber
    auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odomCallback);

    // Inizia a "girare" il nodo per elaborare i messaggi in arrivo
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}


