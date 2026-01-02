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

#include <franka_example_controllers/self_collision_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <algorithm>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
SelfCollisionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
    config.names.push_back(arm_prefixes_[robot_index] + "_" + robot_types_[robot_index] + "/collision_detected");
  }
  return config;

}

controller_interface::InterfaceConfiguration
SelfCollisionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 0; i < robot_types_.size(); i++) {
    for (int j = 1; j <= num_joints; ++j) {
      config.names.push_back(arm_prefixes_[i] + "_" + robot_types_[i] + "_joint" + std::to_string(j) + "/position");
    }
  }

  return config;
}

controller_interface::return_type SelfCollisionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  

  for (size_t i = 0; i < state_interfaces_.size(); ++i){
    current_joint_positions_[i] = state_interfaces_[i].get_optional<double>().value();
  }

  bool collision_found = collision_checker_->checkCollision(current_joint_positions_, print_collisions_);

  double collision_detected = collision_found ? 1.0 : 0.0;
  

  for(size_t i = 0; i < command_interfaces_.size(); ++i) {
    if(!command_interfaces_[i].set_value(collision_detected)){
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to write collision command");
      return controller_interface::return_type::ERROR;
    }
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

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  std::string urdf_xml ;
  auto future_urdf = parameters_client->get_parameters({"robot_description"});
  auto result_urdf = future_urdf.get();
  if (!result_urdf.empty()) {
    urdf_xml = result_urdf[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    return CallbackReturn::ERROR;
  }
  
  std::string srdf_xml ;
  srdf_xml = get_node()->get_parameter("robot_description_semantic").as_string();
  RCLCPP_INFO(get_node()->get_logger(), "Got SRDF file. Is it empty? %s", srdf_xml.empty() ? "Yes" : "No");

  joint_names_.clear();
  for (size_t i = 0; i < robot_types_.size(); i++) {
    for (int j = 1; j <= num_joints; ++j) {
      joint_names_.push_back(arm_prefixes_[i] + "_" + robot_types_[i] + std::to_string(j));
    }
  }

  try{
    RCLCPP_INFO(get_node()->get_logger(), "Loading Pinocchio Model form remote parameters");

    collision_checker_ = std::make_shared<franka_selfcollision::SelfCollisionChecker>(
      urdf_xml,
      srdf_xml,
      security_margin
    );

    // Verify dimensions
    int model_dof = collision_checker_->getDoF();
    if ((size_t)model_dof != joint_names_.size()) {
      RCLCPP_WARN(get_node()->get_logger(), 
        "Mismatch: Controller expects %zu joints, URDF model has %d. (Check if grippers are ignored correctly)", 
        joint_names_.size(), model_dof);
    }
  } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize Collision Checker: %s", e.what());
      return CallbackReturn::ERROR;
  }
  
  current_joint_positions_.resize(joint_names_.size(), 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "Configured Self Collision for %zu joints.", joint_names_.size());

  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SelfCollisionController,
                       controller_interface::ControllerInterface)
