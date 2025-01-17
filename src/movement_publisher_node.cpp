#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/circle_movement.hpp"

class MovementPublisher : public rclcpp::Node {
public:
    MovementPublisher() : Node("movement_publisher"), lx(2.5), az(1.5)
    {
        RCLCPP_INFO(this->get_logger(), "Publisher Node is instantiated.");
        publisher_ = this->create_publisher<custom_interfaces::msg::CircleMovement>("movement_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MovementPublisher::publishCallback, this));
        input_thread_ = std::thread(std::bind(&MovementPublisher::updateMovementData,this));
        input_thread_.detach(); // detach from main thread to run independently.
    }

private:
    void updateMovementData() {
        // rclcpp::ok(): check whether our node is still running.
        while (rclcpp::ok())  {
            std::cout << "Update linear_x and angular_z values (float number), separate by a space > ";
            std::cin >> lx >> az;
            printf("\nYou have updated the data to { linear_x: %.1f, angular_z: %.1f}. Run \"ros2 topic echo /movement_data\" to check the data.\n", lx, az);
        }
    }
    void publishCallback() {
        // Instantiate and assign message
        auto message = custom_interfaces::msg::CircleMovement();
        message.linear_x = lx;
        message.angular_z = az;
        // RCLCPP_INFO(this->get_logger(), "Publishing movement data:\n- linear_x: %.1f\n- angular_z: %.1f",message.linear_x, message.angular_z);
        publisher_->publish(message);
    }
    rclcpp::Publisher<custom_interfaces::msg::CircleMovement>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    float lx, az; // linear-x and angular-z
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto publisher_node = std::make_shared<MovementPublisher>();
    rclcpp::spin(publisher_node);
    rclcpp::shutdown();
}