// Copyright (c) 2024 Franka Robotics GmbH
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

#include <franka_multi_robot_example_controllers/guiding_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

using namespace std::chrono_literals;

namespace franka_multi_robot_example_controllers {

controller_interface::InterfaceConfiguration GuidingController::command_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Grab the command interfaces of the arms and the mobile platform
  config.names.insert(config.names.end(), arm_command_interface_names_.begin(),
                      arm_command_interface_names_.end());
  config.names.insert(config.names.end(), mobile_platform_command_interface_names_.begin(),
                      mobile_platform_command_interface_names_.end());

  return config;
}

controller_interface::InterfaceConfiguration GuidingController::state_interface_configuration()
    const {
  return {};
}

CallbackReturn GuidingController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_ids_ = get_node()->get_parameter("arm_ids").as_string_array();
  for (const auto& arm : arm_ids_) {
    RCLCPP_INFO(get_node()->get_logger(), "Received Arm ID: %s", arm.c_str());
  }

  arm_prefixes_ = get_node()->get_parameter("arm_prefixes").as_string_array();
  for (const auto& prefix : arm_prefixes_) {
    RCLCPP_INFO(get_node()->get_logger(), "Received Arm Prefix: %s", prefix.c_str());
  }

  for (int i = 1; i <= number_of_arm_joints; ++i) {
    for (size_t robot_index = 0; robot_index < arm_ids_.size() - 1; robot_index++) {
      arm_joint_command_indices_.push_back(arm_command_interface_names_.size());
      arm_command_interface_names_.push_back(arm_prefixes_[robot_index] + "_" +
                                             arm_ids_[robot_index] + "_joint" + std::to_string(i) +
                                             "/effort");
    }
  }
  for (int i = 0; i < number_of_wheels; ++i) {
    mobile_platform_command_indices_.push_back(mobile_platform_command_interface_names_.size());
    mobile_platform_command_interface_names_.push_back(mobile_robot_id_ + "_joint" +
                                                       std::to_string(i) + "/effort");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn GuidingController::on_init() {
  try {
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::vector<std::string>>("arm_ids", {});
    auto_declare<std::vector<std::string>>("arm_prefixes", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GuidingController::update(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/) {
  for (const auto& arm_joint_index : arm_joint_command_indices_) {
    command_interfaces_[arm_joint_index].set_value(0.0);
  }

  for (const auto& mobile_platform_index : mobile_platform_command_indices_) {
    command_interfaces_[mobile_platform_index].set_value(0.0);
  }

  for (size_t i = 0; i < command_interfaces_.size(); i++) {
    command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::return_type::OK;
}

}  // namespace franka_multi_robot_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE

PLUGINLIB_EXPORT_CLASS(franka_multi_robot_example_controllers::GuidingController,
                       controller_interface::ControllerInterface)
