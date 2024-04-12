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
#include "plan_helper.hpp"

class myController : public rclcpp::Node {
public:
    myController() : Node("my_controller") {

        // Subscribe to topic: map
        path_subscription = this->create_subscription<nav_msgs::msg::Path>(
            "path", 10, std::bind(&myController::pathCallback, this, std::placeholders::_1));


        path_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive_base_controller/cmd_vel_unstamped", 10);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {

        std::cout << "received path" << std::endl;

        if (path_msg) {
            

            flag_map = true; // map ready

        } else {
            RCLCPP_WARN(this->get_logger(), "Received null map message");
        }
    }


    void publishPath(){
        // Check if all messages are received
        //if (true){
        if (flag_dest && flag_start && flag_dest){


            // publish path
            path_publisher_->publish(path_msg);
            std::cout << "myController: path published!" << std::endl;
            // Reset stored messages
            flag_dest = false;
            flag_start = false;
        }
    }


    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr path_publisher_;



    // stores message from the subscribers

    bool flag_map, flag_start, flag_dest = false; // data not ready
    geometry_msgs::msg::Twist path_msg;




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