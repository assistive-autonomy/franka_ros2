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

// #include <franka_example_controllers/default_robot_behavior_utils.hpp>
// #include <franka_example_controllers/robot_utils.hpp>
#include <franka_multi_robot_example_controllers/joint_velocity_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

namespace franka_multi_robot_example_controllers {

CallbackReturn JointVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_ids_ = get_node()->get_parameter("arm_ids").as_string_array();
  for (const auto& arm : arm_ids_) {
    RCLCPP_INFO(get_node()->get_logger(), "Received Arm ID: %s", arm.c_str());
  }

  // Assumption: The mobile platform is the last robot in the list
  number_of_arms_ = arm_ids_.size() - 1;

  arm_prefixes_ = get_node()->get_parameter("arm_prefixes").as_string_array();
  for (const auto& prefix : arm_prefixes_) {
    RCLCPP_INFO(get_node()->get_logger(), "Received Arm Prefix: %s", prefix.c_str());
  }

  for (int i = 1; i <= number_of_arm_joints; ++i) {
    for (size_t robot_index = 0; robot_index < number_of_arms_; robot_index++) {
      arm_joint_command_indices_.push_back(arm_command_interface_names_.size());
      arm_command_interface_names_.push_back(arm_prefixes_[robot_index] + "_" +
                                             arm_ids_[robot_index] + "_joint" + std::to_string(i) +
                                             "/velocity");
    }
  }

  const size_t summed_indices = arm_command_interface_names_.size();

  for (int i = 0; i < number_of_wheels; ++i) {
    mobile_platform_command_indices_.push_back(summed_indices +
                                               mobile_platform_command_interface_names_.size());
    mobile_platform_command_interface_names_.push_back(mobile_robot_id_ + "_joint" +
                                                       std::to_string(i) + "/velocity");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Grab the command interfaces of the arms and the mobile platform
  config.names.insert(config.names.end(), arm_command_interface_names_.begin(),
                      arm_command_interface_names_.end());
  config.names.insert(config.names.end(), mobile_platform_command_interface_names_.begin(),
                      mobile_platform_command_interface_names_.end());

  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < number_of_arms_; i++) {
    config.names.push_back(arm_prefixes_[i] + "/time");
  }
  return config;
}

CallbackReturn JointVelocityController::on_init() {
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

controller_interface::return_type JointVelocityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    initialization_flag_ = false;
    // Initialize time for each arm
    initial_times_.resize(number_of_arms_);
    for (size_t i = 0; i < number_of_arms_; i++) {
      initial_times_[i] = state_interfaces_[i].get_value();
    }

    mobile_platform_time_ = 0.0;
  }

  // Calculate elapsed time for each robot
  std::vector<double> elapsed_times(arm_ids_.size());
  for (size_t i = 0; i < number_of_arms_; i++) {
    elapsed_times[i] = state_interfaces_[i].get_value() - initial_times_[i];
  }
  elapsed_times[number_of_arms_] = mobile_platform_time_;
  mobile_platform_time_ += 0.001;

  rclcpp::Duration time_max(2.0, 0.0);
  double omega_max = 1.8;

  // Calculate velocities for each robot based on its own time
  std::vector<double> omegas(arm_ids_.size());
  for (size_t i = 0; i < arm_ids_.size(); i++) {
    double cycle = std::floor(std::pow(
        -1.0,
        (elapsed_times[i] - std::fmod(elapsed_times[i], time_max.seconds())) / time_max.seconds()));
    omegas[i] = cycle * omega_max / 2.0 *
                (1.0 - std::cos(2.0 * M_PI / time_max.seconds() * elapsed_times[i]));
  }

  std::vector<int> joint_indices;
  std::vector<int> joint_number = {3, 4};
  for (int i = 0; i < number_of_arm_joints; i++) {
    for (size_t j = 0; j < number_of_arms_; j++) {
      int index = joint_number[i] * arm_ids_.size() + j;
      joint_indices.push_back(index);
    }
  }

  // + the mobile platform
  joint_indices.insert(joint_indices.end(), mobile_platform_command_indices_.begin(),
                       mobile_platform_command_indices_.end());

  // Set velocities for each joint using the corresponding robot's omega
  for (size_t i = 0; i < command_interfaces_.size(); i++) {
    if (std::find(joint_indices.begin(), joint_indices.end(), i) != joint_indices.end()) {
      // Calculate which robot this joint belongs to. i % number_of_arms_ will
      // give the arm index while the mobile platform will be number_of_arms_.
      // Command index 14 + 15 are the wheels.
      size_t robot_index =
          i < number_of_arm_joints * number_of_arms_ ? i % number_of_arms_ : number_of_arms_;
      auto velocity = omegas[robot_index] * 0.1;
      command_interfaces_[i].set_value(velocity);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace franka_multi_robot_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE

PLUGINLIB_EXPORT_CLASS(franka_multi_robot_example_controllers::JointVelocityController,
                       controller_interface::ControllerInterface)
