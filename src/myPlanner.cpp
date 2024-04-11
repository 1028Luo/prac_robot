// Written by Jiexing Luo, email:jiexingluo@Outlook.com
//
// This file contains the path planner myPlanner for the project prac_robot
// By default it uses a simple A* as its path planning algorithm 
//
// myPlanner subscribes to three topics:
//      map - to get map information
//      goal_pose - to get any goal pose made in RVIZ
//      amcl_pose - to get current pose, covariance discarded 
// 
// myPlanner publishes the generated path to myController


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" // map
#include "nav_msgs/msg/path.hpp" // map

#include "geometry_msgs/msg/pose_stamped.hpp" // goal_pose
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" //AMCL
#include "simple_Astar.hpp"


class myPlanner : public rclcpp::Node {
public:
    myPlanner() : Node("my_planner") {


        // Subscribe to topic: map
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&myPlanner::mapCallback, this, std::placeholders::_1));

        // Subscribe to topic: goal_pose
        goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&myPlanner::goalCallback, this, std::placeholders::_1));

        // Subscribe to topic: AMCL_pose
        AMCL_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&myPlanner::AMCL_Callback, this, std::placeholders::_1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {

        std::cout << "received map" << std::endl;

        if (map_msg) {
            
            // access the info field
            std::cout << "Map Info: ";
            std::cout << "Width: " << map_msg->info.width << ", ";
            std::cout << "Height: " << map_msg->info.height << ", ";
            std::cout << "Resolution: " << map_msg->info.resolution << std::endl;

            // stores data in 2d array, initialise all to 0
            std::vector<std::vector<int>> map_temp(map_msg->info.height, std::vector<int>(map_msg->info.width,0));

            for (size_t i =0 ; i < map_msg->info.height; i++){
                for (size_t j = 0; j < map_msg->info.width; j++){
                    auto data_idx = i * map_msg->info.width + j;
                    map_temp[i][j] = map_msg->data[data_idx];
                }
            }
            
            std::cout << "2d occupancy grid made" << std::endl;
            std::cout << std::endl;
            map = map_temp;

            flag_map = true; // map ready


            // print map
            printMap(map);

        } else {
            RCLCPP_WARN(this->get_logger(), "Received null map message");
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg) {

        std::cout << "received goal pose" << std::endl;
        if (goal_pose_msg) {

            std::cout << "Goal pose: ";
            std::cout << "X: " << goal_pose_msg->pose.position.x << std::endl;
            std::cout << "Y: " << goal_pose_msg->pose.position.y << std::endl;
            std::cout << "rot Z: " << goal_pose_msg->pose.orientation.z << std::endl;
            std::cout << std::endl;

            flag_dest = true;
            publishPath();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null goal message");
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

            start.x = AMCL_pose_msg->pose.pose.position.x;
            start.y = AMCL_pose_msg->pose.pose.position.y;
            
            flag_start = true; // start ready
            publishPath();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null goal message");
        }
    }

    void publishPath(){
        // Check if all messages are received
        if (true){
        //if (flag_dest && flag_start && flag_dest){
            // Combine data from all messages as needed

            // fill information in path
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = 1;
            pose.pose.position.y = 1;
            nav_msgs::msg::Path path_msg;
            path_msg.poses.push_back(pose);

            //auto combinedData = combineMessages(msg1_, msg2_, msg3_);
            // Publish combined data
            path_publisher_->publish(path_msg);

            // Reset stored messages
            flag_dest = false;
            flag_start = false;
        }
    }


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr AMCL_pose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;



    // stores message from the subscribers
    std::vector<std::vector<int>> map;
    point start;
    point dest;
    bool flag_map, flag_start, flag_dest = false; // data not ready



void printMap(const std::vector<std::vector<int>>& map_temp) {
    for (const auto& row : map_temp) {
        for (int value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
}


};

int main(int argc, char** argv) {
    std::cout << "running myPlanner" << std::endl;
    
    std::cout << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<myPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}