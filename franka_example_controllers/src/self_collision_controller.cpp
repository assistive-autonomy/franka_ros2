// Copyright (c) 2026 Franka Robotics GmbH
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

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_example_controllers/self_collision_controller.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <thread>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
SelfCollisionController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
SelfCollisionController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type SelfCollisionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    size_t data_index = state_interface_map_[i];
    current_joint_positions_[data_index] = state_interfaces_[i].get_optional<double>().value();
  }

  bool collision_found =
      collision_checker_->checkCollision(current_joint_positions_, print_collisions_);

  if (collision_found) {
    RCLCPP_FATAL(get_node()->get_logger(), "EXTERNAL COLLISION DETECTED VIA ROS2!");
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SelfCollisionController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("arm_prefixes", {});
    auto_declare<std::vector<std::string>>("robot_types", {});

    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_description_semantic", "");
    auto_declare<double>("security_margin", 0.045);

    auto_declare<bool>("print_collisions", false);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SelfCollisionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_prefixes_ = get_node()->get_parameter("arm_prefixes").as_string_array();
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  double security_margin = get_node()->get_parameter("security_margin").as_double();
  print_collisions_ = get_node()->get_parameter("print_collisions").as_bool();

  std::string urdf_xml;
  urdf_xml = get_node()->get_parameter("robot_description").as_string();
  RCLCPP_INFO(get_node()->get_logger(), "Got URDF file. Is it empty? %s",
              urdf_xml.empty() ? "Yes" : "No");

  std::string srdf_xml;
  srdf_xml = get_node()->get_parameter("robot_description_semantic").as_string();
  RCLCPP_INFO(get_node()->get_logger(), "Got SRDF file. Is it empty? %s",
              srdf_xml.empty() ? "Yes" : "No");

  try {
    RCLCPP_INFO(get_node()->get_logger(), "Loading Pinocchio Model form remote parameters");

    collision_checker_ = std::make_shared<franka_selfcollision::SelfCollisionChecker>(
        urdf_xml, srdf_xml, security_margin);

    std::vector<std::string> pinocchio_names = collision_checker_->getModelJointNames();
    joint_names_.clear();
    for (const auto& name : pinocchio_names) {
      if (name == "universe")
        continue;
      joint_names_.push_back(name);
    }

    // Verify dimensions
    int model_dof = collision_checker_->getDoF();
    if ((size_t)model_dof != joint_names_.size()) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Mismatch: Controller expects %zu joints, URDF model has %d. (Check if grippers "
                  "are ignored correctly)",
                  joint_names_.size(), model_dof);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize Collision Checker: %s", e.what());
    return CallbackReturn::ERROR;
  }

  current_joint_positions_.resize(joint_names_.size(), 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "Configured Self Collision for %zu joints.",
              joint_names_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn SelfCollisionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  state_interface_map_.clear();
  state_interface_map_.resize(state_interfaces_.size());

  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    std::string full_name = state_interfaces_[i].get_name();
    std::string base_name = full_name.substr(0, full_name.find("/position"));

    auto it = std::find(joint_names_.begin(), joint_names_.end(), base_name);
    if (it != joint_names_.end()) {
      state_interface_map_[i] = std::distance(joint_names_.begin(), it);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not find match for joint: %s",
                   base_name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SelfCollisionController,
                       controller_interface::ControllerInterface)
