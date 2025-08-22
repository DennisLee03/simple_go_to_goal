#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "demo_diff_drive_pkg/action/simple_go_to_goal.hpp"
#include "demo_diff_drive_pkg/PID_control.hpp"

typedef demo_diff_drive_pkg::action::SimpleGoToGoal SimpleGoToGoalAction;
typedef rclcpp_action::ServerGoalHandle<SimpleGoToGoalAction> GoalHandle;

const double DISTANCE_THRESHHOLD = 0.1;

class SimpleGoToGoalActionServer: public rclcpp::Node
{
    public:
        SimpleGoToGoalActionServer():
        Node("simple_go_to_goal_action_server_node"),
        linear_controller_(0.5, 0.01, 0.2, 2.0, 1.0),
        angular_controller_(0.8, 0.02, 0.1, 2.0, 2.1)
        {
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10,
                std::bind(&SimpleGoToGoalActionServer::odom_callback, this, std::placeholders::_1)
            );

            vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            action_server_ = rclcpp_action::create_server<SimpleGoToGoalAction>(
                this,
                "simple_go_to_goal",
                std::bind(&SimpleGoToGoalActionServer::handle_goal, this,
                    std::placeholders::_1, std::placeholders::_2),
                std::bind(&SimpleGoToGoalActionServer::handle_cancel, this,
                    std::placeholders::_1),
                std::bind(&SimpleGoToGoalActionServer::handle_accepted, this,
                    std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "Simple-Go-To-Goal Action Server Started\n");
        }
    private:
        // Member variables for the robot's current state
        double current_x_, current_y_, current_yaw_;
        // Member variables for the PID controllers
        PID_control linear_controller_, angular_controller_;
        // Mutex to protect shared data between threads
        std::mutex mutex_;
        std::condition_variable cv_;
        bool new_odom_data_ = false;

        // Action-related variables
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
        rclcpp_action::Server<SimpleGoToGoalAction>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const SimpleGoToGoalAction::Goal> goal)
        {
            (void)uuid;
            RCLCPP_INFO(this->get_logger(), "Received goal point: (%.2f, %.2f)\n", goal->x, goal->y);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal\n");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            std::thread{std::bind(&SimpleGoToGoalActionServer::execute, this, goal_handle)}.detach();
        }

        void execute(const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal\n");
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<SimpleGoToGoalAction::Feedback>();
            auto result = std::make_shared<SimpleGoToGoalAction::Result>();

            // Get initial state
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]{ return new_odom_data_; });
            new_odom_data_ = false;

            // Use a fixed rate for the control loop
            rclcpp::Rate rate(50); // 50 Hz, a good standard for robot control
            
            // We now update distance and angle inside the loop
            double distance_to_goal;
            double angle_to_goal;
            double angle_diff;
            auto twist_msg = geometry_msgs::msg::Twist();
            
            bool goal_reached = false;
            
            while (!goal_reached)
            {
                // Check for cancellation
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled\n");
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    vel_publisher_->publish(twist_msg);
                    return;
                }

                // Wait for new odometry data
                cv_.wait(lock, [this]{ return new_odom_data_; });
                new_odom_data_ = false;

                // Update robot's state and errors
                distance_to_goal = std::sqrt(std::pow(goal->x - current_x_, 2) + std::pow(goal->y - current_y_, 2));
                angle_to_goal = std::atan2(goal->y - current_y_, goal->x - current_x_);
                angle_diff = angle_to_goal - current_yaw_;
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
                
                // Calculate and apply PID outputs
                double linear_vel = linear_controller_.pid_calc(0.0, -distance_to_goal);
                double angular_vel = angular_controller_.pid_calc(0.0, -angle_diff);

                twist_msg.linear.x = linear_vel;
                twist_msg.angular.z = angular_vel;

                // Publish Twist command
                vel_publisher_->publish(twist_msg);

                // Publish feedback
                feedback->distance_to_goal = distance_to_goal;
                feedback->linear_velocity = linear_vel;
                feedback->angular_velocity = angular_vel;
                goal_handle->publish_feedback(feedback);
                
                // Check stop condition
                if (distance_to_goal < DISTANCE_THRESHHOLD) {
                    goal_reached = true;
                }

                lock.unlock(); // Release the lock before sleeping
                rate.sleep();
                lock.lock(); // Re-acquire the lock before the next iteration
            }
            
            // Goal reached
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            vel_publisher_->publish(twist_msg);

            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded\n");
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_); // std::lock_guard is safer
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
            new_odom_data_ = true;
            cv_.notify_one(); // Use notify_one for single-waiter threads
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoToGoalActionServer>());
    rclcpp::shutdown();
    return 0;
}