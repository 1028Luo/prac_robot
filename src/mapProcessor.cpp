#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class mapProcessor : public rclcpp::Node {
public:
    mapProcessor() : Node("map_processor") {
        // Subscribe to the /maps topic
        subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/maps", 10, std::bind(&mapProcessor::mapCallback, this, std::placeholders::_1));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
        // Process the map data received
        // You can access the map data using map_msg->data, map_msg->info, etc.
        // Check if map_msg is not null

        if (map_msg) {
            // Print the first 10 elements of the map data
            std::cout << "Map Data: ";
            for (size_t i = 0; i < 10; ++i) {
                std::cout << static_cast<int>(map_msg->data[i]) << " ";
            }
            std::cout << std::endl;
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null map message");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mapProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}