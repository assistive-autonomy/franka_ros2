// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_mobile_example_controllers/joint_velocity_example_controller.hpp>

#include <exception>
#include <string>

namespace franka_mobile_example_controllers {

controller_interface::InterfaceConfiguration
JointVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 0; i < num_joints; ++i) {
    config.names.push_back(mobile_robot_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityExampleController::state_interface_configuration() const {
  return {};
}

controller_interface::return_type JointVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  static double total_time = 0.0;
  total_time += period.seconds();
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(std::sin(total_time));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  mobile_robot_id_ = get_node()->get_parameter("mobile_robot_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_init() {
  try {
    auto_declare<std::string>("mobile_robot_id", "tmr");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_mobile_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_mobile_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerInterface)
