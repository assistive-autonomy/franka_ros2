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

#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include "franka_multi_robot_example_controllers/visibility_control.h"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_multi_robot_example_controllers {

/**
 * The joint velocity controller sends sine waves as joint velocities, so that
 * the robot moves back and forth (arms and mobile platform).
 * Important: Assumption that the mobile platform is the last robot in the list
 */
class JointVelocityController : public controller_interface::ControllerInterface {
 public:
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  std::string mobile_robot_id_ = "tmr";
  std::vector<std::string> arm_ids_;
  std::vector<std::string> arm_prefixes_;
  const int number_of_arm_joints = 7;
  const int number_of_wheels = 2;
  size_t number_of_arms_;

  // Parameters for quickly accessing the command interfaces
  std::vector<size_t> arm_joint_command_indices_;
  std::vector<std::string> arm_command_interface_names_;
  std::vector<size_t> mobile_platform_command_indices_;
  std::vector<std::string> mobile_platform_command_interface_names_;

  // Specific parameters for controller type
  std::vector<double> initial_times_;
  double mobile_platform_time_{0};
  bool initialization_flag_{true};
};
}  // namespace franka_multi_robot_example_controllers
