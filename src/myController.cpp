// Written by Jiexing Luo, email:jiexingluo@Outlook.com
//
// This file contains the path planner myController for the project prac_robot
// By default it uses a simple A* as its path planning algorithm 
//
// myController subscribes to three topics:
//      map - to get map information
//      goal_pose - to get any goal pose made in RVIZ
//      amcl_pose - to get current pose, covariance discarded 
// 
// myController publishes the generated path to myController


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" //AMCL
#include "nav_msgs/msg/odometry.hpp" //odom
#include "plan_helper.hpp"
#include "simple_LQR.hpp"
//#include "tf2/LinearMath/Quaternion.h"
//#include "tf2/LinearMath/Matrix3x3.h"


class myController : public rclcpp::Node {
public:
    myController() : Node("my_controller") {

        // Subscribe to topic: map
        path_subscription = this->create_subscription<nav_msgs::msg::Path>(
            "path", 10, std::bind(&myController::pathCallback, this, std::placeholders::_1));
        
        // Subscribe to topic: AMCL_pose
        AMCL_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&myController::AMCL_Callback, this, std::placeholders::_1));

        control_trigger_ = this->create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&myController::controlLoop, this));

        // publish to topic: diff_drive_base_controller/cmd_vel_unstamped
        path_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive_base_controller/cmd_vel_unstamped", 10);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {

        std::cout << "received path" << std::endl;

        if (path_msg) {
            path_msg_ = *path_msg;
            myLQR.initLQR();
            flag_path = true;
            //controlLoop();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null map message");
        }
    }

    void AMCL_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr AMCL_pose_msg) {

        std::cout << "received AMCL pose" << std::endl;
        if (AMCL_pose_msg) {

            std::cout << "Current AMCL pose: ";
            std::cout << "X: " << AMCL_pose_msg->pose.pose.position.x << std::endl;
            std::cout << "Y: " << AMCL_pose_msg->pose.pose.position.y << std::endl;
            std::cout << "rot Z: " << AMCL_pose_msg->pose.pose.orientation.z << std::endl;
            std::cout << std::endl;
            //point temp = pose2map(AMCL_pose_msg->pose.pose.position.x, AMCL_pose_msg->pose.pose.position.y);

            currState_AMCL.x = AMCL_pose_msg->pose.pose.position.x;
            currState_AMCL.y = AMCL_pose_msg->pose.pose.position.y;
            currState_AMCL.yaw = AMCL_pose_msg->pose.pose.orientation.z;

            flag_curr_pose = true; // start ready
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null pose message");
        }
    }

    void controlLoop(){

        if (flag_curr_pose && !flag_reached_target && flag_path){

            std::cout << "myController: in control" << std::endl;

            State desiredState;
            std::cout << "path size:" << path_msg_.poses.size() << std::endl;

            std::cout << "Following point :" << path_idx << std::endl;
            
            // track a waypoint in path
            auto s = path_msg_.poses[path_idx];
            desiredState.x = s.pose.position.x;
            desiredState.y = s.pose.position.y;
            desiredState.yaw = 1;

            // get control input
            ControlInput input = myLQR.generateControlInput(currState_AMCL, desiredState, dt_);
            
            // cap the input
            if (input.linear_vel_x > vel_linear_x_cap) {
                input.linear_vel_x = vel_linear_x_cap;
            } else if (input.linear_vel_x < -vel_linear_x_cap) {
                input.linear_vel_x = -vel_linear_x_cap;
            }
            
            if (input.angular_vel_z > vel_angualr_z_cap) {
                input.angular_vel_z = vel_angualr_z_cap;
            } else if (input.angular_vel_z < -vel_angualr_z_cap) {
                input.angular_vel_z = -vel_angualr_z_cap;
            }
            
            cmd_msg.linear.x = input.linear_vel_x;
            cmd_msg.angular.z = input.angular_vel_z;


            // publish control command
            std::cout << "myController: sending control input" << std::endl;
            std::cout << "Linear X:" << cmd_msg.linear.x << std::endl;
            std::cout << "Angular Z:" << cmd_msg.angular.z << std::endl;

            path_publisher_->publish(cmd_msg);

            // if this way point has been reached, move to next waypoint
             std::cout << "curr error:" << compute_error(currState_AMCL, desiredState) << std::endl;
            if(compute_error(currState_AMCL, desiredState) <= tolerance) {
                
                path_idx = path_idx + 4;
                if (path_idx > path_msg_.poses.size()){
                    path_idx = path_msg_.poses.size()-1;
                }
            }
        }
    }

    double compute_error(State curr, State desired) {
        auto x_squared = (curr.x - desired.x) * (curr.x - desired.x);
        auto y_squared = (curr.y - desired.y) * (curr.y - desired.y);

        return sqrt(x_squared + y_squared);
    }




    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr AMCL_pose_subscription_;

    rclcpp::TimerBase::SharedPtr control_trigger_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr path_publisher_;



    // stores message from the subscribers
    double dt_ = 0.05;
    bool flag_curr_pose = false;
    bool flag_reached_target = false;
    bool flag_path = false;
    nav_msgs::msg::Path path_msg_;
    geometry_msgs::msg::Twist cmd_msg;
    LQR myLQR;
    State currState_AMCL;
    State currState_odom;

    int path_idx = 0;
    double tolerance = 1.5;
    double vel_linear_x_cap = 0.5;
    double vel_angualr_z_cap = 0.5;
};

int main(int argc, char** argv) {
    std::cout << "myController is running" << std::endl;
    
    std::cout << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<myController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}