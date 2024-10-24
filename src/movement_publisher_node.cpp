#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/circle_movement.hpp"

class MovementPublisher : public rclcpp::Node {
public:
    MovementPublisher() : Node("movement_publisher")
    {
        publisher_ = this->create_publisher<custom_interfaces::msg::CircleMovement>("movement_data", 10);
    }

private:
    rclcpp::Publisher<custom_interfaces::msg::CircleMovement>::SharedPtr publisher_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto publisher_node = std::make_shared<MovementPublisher>();
    rclcpp::spin(publisher_node);
    rclcpp::shutdown();
}