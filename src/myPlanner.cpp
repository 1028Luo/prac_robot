#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"




class myPlanner : public rclcpp::Node {
public:
    myPlanner() : Node("my_planner") {


        // Subscribe to the first topic
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&myPlanner::mapCallback, this, std::placeholders::_1));


        // Subscribe to the /map topic
        goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&myPlanner::goalCallback, this, std::placeholders::_1));


    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {

        std::cout << "received map" << std::endl;
        if (map_msg) {
            
            // Access the info field
            std::cout << "Map Info: ";
            std::cout << "Width: " << map_msg->info.width << ", ";
            std::cout << "Height: " << map_msg->info.height << ", ";
            std::cout << "Resolution: " << map_msg->info.resolution << std::endl;


            // stores data in 2d array, initialise all to 0
            std::vector<std::vector<int>> map_2d(map_msg->info.height, std::vector<int>(map_msg->info.width,0));

            for (size_t i =0 ; i < map_msg->info.height; i++){
                for (size_t j = 0; j < map_msg->info.width; j++){
                    auto data_idx = i * map_msg->info.width + j;
                    map_2d[i][j] = map_msg->data[data_idx];
                }
            }
            
            std::cout << "2d occupancy grid made" << std::endl;
            std::cout << std::endl;

        } else {
            RCLCPP_WARN(this->get_logger(), "Received null map message");
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg) {

        std::cout << "received goal pose" << std::endl;
        if (goal_pose_msg) {
            std::cout << "Goal pose: ";
            std::cout << "X: " << goal_pose_msg->pose.position.x << std::endl;

        } else {
            RCLCPP_WARN(this->get_logger(), "Received null goal message");
        }
    }



    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
};

int main(int argc, char** argv) {
    std::cout << "running myPlanner" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<myPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}