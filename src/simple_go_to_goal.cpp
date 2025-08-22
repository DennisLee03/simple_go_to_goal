#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "demo_diff_drive_pkg/PID_control.hpp"

class SimpleGoToGoalNode: public rclcpp::Node
{
    public:
        SimpleGoToGoalNode(): 
        Node("simple_go_to_goal_node"),
        // Kp, Ki, Kd, max_integral, max_output(cmd_vel)
        linear_controller_(0.5, 0.01, 0.2, 2.0, 1.0), 
        angular_controller_(0.8, 0.02, 0.1, 2.0, 2.1)
        {
            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, 
                std::bind(&SimpleGoToGoalNode::odom_callback, this, std::placeholders::_1)
            );

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            std::cout << "Enter your destination(in the format of \'x y\'): ";
            std::cin >> this->goal_x_ >> goal_y_;

            RCLCPP_INFO(this->get_logger(), "Goal set to: (%.2f, %.2f)", goal_x_, goal_y_);
        }

    private:

        // using pid control
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            // get current position and angle(response): current_x_, current_y_, current_yaw_
            current_x_ = msg->pose.pose.position.x;
            current_y_ = msg->pose.pose.position.y;
            tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_yaw_ = yaw;

            double distance_to_goal = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
            double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
            double angle_diff = angle_to_goal - current_yaw_;

            // Normalize angle_diff to the range [-pi, pi]
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            // Calculate the PID outputs for both linear and angular velocity
            double linear_output = linear_controller_.pid_calc(0.0, -distance_to_goal); // target is 0, response is -distance
            double angular_output = 1.0*angle_diff; // angular_controller_.pid_calc(0.0, -angle_diff); // target is 0, response is -angle

            auto twist_msg = geometry_msgs::msg::Twist();

            // Apply the PID outputs to your Twist message
            // Use the calculated outputs for the linear and angular commands
            twist_msg.linear.x = linear_output;
            twist_msg.angular.z = angular_output;

            // Add a stop condition for when the robot is close to the goal
            if (distance_to_goal < 0.1) {
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                publisher_->publish(twist_msg);
                rclcpp::shutdown();
            }

            publisher_->publish(twist_msg);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        double goal_x_, goal_y_;
        double current_x_, current_y_, current_yaw_;
        PID_control linear_controller_, angular_controller_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoToGoalNode>());
    rclcpp::shutdown();
    return 0;
}