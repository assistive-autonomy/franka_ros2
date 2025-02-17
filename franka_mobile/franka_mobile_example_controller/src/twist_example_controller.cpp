#include <franka_mobile_example_controllers/twist_example_controller.hpp>

using namespace std::chrono_literals;

namespace franka_mobile_example_controllers {

TwistExampleController::TwistExampleController() : Node("twist_example_controller") {
  publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 1);
  start_time_ = this->now().seconds();
  auto timer_callback = [this]() -> void {
    auto message = geometry_msgs::msg::TwistStamped();
    double elapsed_time = this->now().seconds() - this->start_time_;
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";
    message.twist.linear.x = 0.0;
    message.twist.linear.y = 0.0;
    message.twist.linear.z = 0.0;
    message.twist.angular.x = 0.0;
    message.twist.angular.y = 0.0;
    message.twist.angular.z = std::sin(elapsed_time);
    publisher_->publish(message);
  };
  timer_ = this->create_wall_timer(50ms, timer_callback);
}
}  // namespace franka_mobile_example_controllers

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_mobile_example_controllers::TwistExampleController>());
  rclcpp::shutdown();
  return 0;
}
