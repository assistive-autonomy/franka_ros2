#include "franka_mobile_example_controllers/mobile_cartesian_velocity_with_ik_example_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace franka_mobile_example_controllers {

controller_interface::CallbackReturn MobileCartesianVelocityWithIkExampleController::on_init() {
  RCLCPP_INFO(rclcpp::get_logger("MobileCartesianVelocityWithIkExampleController"),
              "Controller initialized");

  // Set wheel positions relative to base (example from URDF)
  wheel_positions_ << 0.3, -0.2,  // front
      -0.3, 0.2;                  // rear

  steering_angles_.setZero();
  wheel_velocities_.setZero();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MobileCartesianVelocityWithIkExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
      mobile_robot_type_ + "_joint_0/position",  // front steering
      mobile_robot_type_ + "_joint_1/velocity",  // front wheel
      mobile_robot_type_ + "_joint_2/position",  // rear steering
      mobile_robot_type_ + "_joint_3/velocity"   // rear wheel
  };
  return config;
}

controller_interface::InterfaceConfiguration
MobileCartesianVelocityWithIkExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

void MobileCartesianVelocityWithIkExampleController::computeSwerveIK(double vx,
                                                                     double vy,
                                                                     double wz) {
  Eigen::ArrayXd x = wheel_positions_.col(0);
  Eigen::ArrayXd y = wheel_positions_.col(1);

  Eigen::ArrayXd vx_wheel = vx - wz * y;
  Eigen::ArrayXd vy_wheel = vy + wz * x;

  wheel_velocities_ = ((vx_wheel.square() + vy_wheel.square()).sqrt()) / wheel_radius_;
  steering_angles_ =
      vy_wheel.binaryExpr(vx_wheel, [](double vy, double vx) { return std::atan2(vy, vx); });

  // Wrap angles 0 -> 2pi
  for (int i = 0; i < kNumberOfWheels; ++i) {
    if (steering_angles_(i) < 0.0)
      steering_angles_(i) += 2.0 * M_PI;
    commands_[i].steering_angle = steering_angles_(i);
    commands_[i].wheel_velocity = wheel_velocities_(i);
  }
}

controller_interface::return_type MobileCartesianVelocityWithIkExampleController::update(
    const rclcpp::Time&,
    const rclcpp::Duration& period) {
  elapsed_time_ += period.seconds();

  // Sinusoidal Cartesian velocities
  double vx = vx_amp_ * std::sin(2.0 * M_PI * freq_ * elapsed_time_);
  double vy = vy_amp_ * std::sin(2.0 * M_PI * freq_ * elapsed_time_);
  double wz = wz_amp_ * std::sin(2.0 * M_PI * freq_ * elapsed_time_);

  // Compute swerve IK
  computeSwerveIK(vx, vy, wz);

  // Send to command interfaces
  command_interfaces_[0].set_value(commands_[0].steering_angle);
  command_interfaces_[1].set_value(commands_[0].wheel_velocity);
  command_interfaces_[2].set_value(commands_[1].steering_angle);
  command_interfaces_[3].set_value(commands_[1].wheel_velocity);

  return controller_interface::return_type::OK;
}

}  // namespace franka_mobile_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_mobile_example_controllers::MobileCartesianVelocityWithIkExampleController,
    controller_interface::ControllerInterface)
