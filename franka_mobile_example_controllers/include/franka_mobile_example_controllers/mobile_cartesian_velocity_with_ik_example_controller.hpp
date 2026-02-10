#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace franka_mobile_example_controllers {

class MobileCartesianVelocityWithIkExampleController
    : public controller_interface::ControllerInterface {
 public:
  MobileCartesianVelocityWithIkExampleController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  // Robot params
  std::string mobile_robot_type_ = "tmrv0_2";
  static constexpr size_t kNumberOfWheels = 2;
  double wheel_radius_ = 0.05;                                 // m
  Eigen::Matrix<double, kNumberOfWheels, 2> wheel_positions_;  // x, y

  // Sinusoidal trajectory
  double elapsed_time_ = 0.0;
  double freq_ = 0.5;  // Hz
  double vx_amp_ = 0.2;
  double vy_amp_ = 0.0;
  double wz_amp_ = M_PI / 8.0;  // rad/s

  // IK state
  Eigen::Array<double, kNumberOfWheels, 1> steering_angles_;
  Eigen::Array<double, kNumberOfWheels, 1> wheel_velocities_;

  struct WheelCommand {
    double steering_angle = 0.0;
    double wheel_velocity = 0.0;
  };
  std::vector<WheelCommand> commands_{kNumberOfWheels};

  // IK computation
  void computeSwerveIK(double vx, double vy, double wz);
};

}  // namespace franka_mobile_example_controllers
