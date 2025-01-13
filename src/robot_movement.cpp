#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <std_msgs/msg/float64.hpp>
#include "std_srvs/srv/set_bool.hpp"

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_x_pub;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_y_pub;

rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_service;

bool allow_movement = true;

double current_x = 0.0;
double current_y = 0.0;
double feet_x = 0.0;
double feet_y = 0.0;


void feetPosition() {
	feet_x = current_x * 3.28;
	feet_y = current_y * 3.28;

	std_msgs::msg::Float64 feet_x_pos_msg;
	feet_x_pos_msg.data = feet_x;
	position_x_pub->publish(feet_x_pos_msg);
	
	std_msgs::msg::Float64 feet_y_pos_msg;
	feet_y_pos_msg.data = feet_y;
	position_y_pub->publish(feet_y_pos_msg); 
}

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{	
	if (!allow_movement) {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        pub->publish(stop_msg);
        return;
    }
	
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
    
    feetPosition();
}

void controlMovementService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
	
	allow_movement = request->data;
	response->success = true;
    response->message = allow_movement ? "Robot movement enabled" : "Robot movement disabled";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Initialize the node
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_robot_node");

    // Publisher
    pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Publishers for the position
    position_x_pub = node->create_publisher<std_msgs::msg::Float64>("/robot_x_position", 10);
    position_y_pub = node->create_publisher<std_msgs::msg::Float64>("/robot_y_position", 10);

    // Subscriber
    auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odomCallback);
        
    // Service 
	control_service = node->create_service<std_srvs::srv::SetBool>(
        "control_movement",
        &controlMovementService);

    // Inizia a "girare" il nodo per elaborare i messaggi in arrivo
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}


