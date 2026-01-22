// Copyright (c) 2025 Franka Robotics GmbH
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

#include <franka_example_controllers/mobile_fr3_duo_joint_impedance_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Add command interfaces for dual arms
  for (int i = 1; i <= num_joints; ++i) {
    for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
      config.names.push_back(arm_prefixes_[robot_index] + "_" + robot_types_[robot_index] +
                             "_joint" + std::to_string(i) + "/effort");
    }
  }

  // Add command interfaces for mobile base (GPIO-style cartesian velocity)
  if (enable_mobile_base_) {
    config.names.push_back("vx/cartesian_velocity");
    config.names.push_back("vy/cartesian_velocity");
    config.names.push_back("vz/cartesian_velocity");
    config.names.push_back("wx/cartesian_velocity");
    config.names.push_back("wy/cartesian_velocity");
    config.names.push_back("wz/cartesian_velocity");
  }

  return config;
}

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Add state interfaces for dual arms
  for (int i = 1; i <= num_joints; ++i) {
    for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
      config.names.push_back(arm_prefixes_[robot_index] + "_" + robot_types_[robot_index] +
                             "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_prefixes_[robot_index] + "_" + robot_types_[robot_index] +
                             "_joint" + std::to_string(i) + "/velocity");
    }
  }

  return config;
}

controller_interface::return_type MobileFr3DuoJointImpedanceExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  // Update joint states for dual arms
  updateJointStates();

  elapsed_time_ = elapsed_time_ + period.seconds();

  // Control dual arms with joint impedance
  for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
    Vector7d q_goal = initial_q_[robot_index];

    double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * elapsed_time_));
    q_goal(3) += delta_angle;
    q_goal(4) += delta_angle;

    const double kAlpha = 0.99;
    dq_filtered_[robot_index] =
        (1 - kAlpha) * dq_filtered_[robot_index] + kAlpha * dq_[robot_index];
    Vector7d tau_d_calculated = k_gains_.cwiseProduct(q_goal - q_[robot_index]) +
                                d_gains_.cwiseProduct(-dq_filtered_[robot_index]);

    for (int i = 0; i < num_joints; ++i) {
      if (!command_interfaces_[i * robot_types_.size() + robot_index].set_value(
              tau_d_calculated(i))) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value");
      }
    }
  }

  // Control mobile base with cartesian velocity
  if (enable_mobile_base_) {
    updateMobileBaseCommand(period);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("robot_types", std::vector<std::string>{});
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<std::string>>("arm_prefixes", {});
    auto_declare<bool>("enable_mobile_base", true);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  enable_mobile_base_ = get_node()->get_parameter("enable_mobile_base").as_bool();

  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  arm_prefixes_ = get_node()->get_parameter("arm_prefixes").as_string_array();
  for (const auto& prefix : arm_prefixes_) {
    RCLCPP_INFO(get_node()->get_logger(), "Received Arm Prefix: %s", prefix.c_str());
  }

  for (auto& dq : dq_filtered_) {
    dq.setZero();
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Ensure vectors are sized for the number of robots
  q_.resize(robot_types_.size(), Vector7d::Zero());
  dq_.resize(robot_types_.size(), Vector7d::Zero());
  dq_filtered_.resize(robot_types_.size(), Vector7d::Zero());
  initial_q_.resize(robot_types_.size(), Vector7d::Zero());

  updateJointStates();
  for (auto& dq : dq_filtered_) {
    dq.setZero();
  }
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  // Find and store mobile base command interface indices
  if (enable_mobile_base_) {
    // The mobile base interfaces are at the end of command_interfaces_
    // after all the arm joint effort interfaces
    size_t arm_interfaces_count = num_joints * robot_types_.size();

    // Expected order: vx, vy, vz, wx, wy, wz
    mobile_vx_index_ = arm_interfaces_count + 0;
    mobile_vy_index_ = arm_interfaces_count + 1;
    mobile_vz_index_ = arm_interfaces_count + 2;
    mobile_wx_index_ = arm_interfaces_count + 3;
    mobile_wy_index_ = arm_interfaces_count + 4;
    mobile_wz_index_ = arm_interfaces_count + 5;

    RCLCPP_INFO(get_node()->get_logger(),
                "Mobile base interfaces assigned (vx, vy, vz, wx, wy, wz)");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void MobileFr3DuoJointImpedanceExampleController::updateJointStates() {
  for (size_t robot_index = 0; robot_index < robot_types_.size(); robot_index++) {
    for (auto i = 0; i < num_joints; ++i) {
      // Joint i, robot robot_index
      size_t pos_index = i * robot_types_.size() * 2 + robot_index * 2;
      size_t vel_index = pos_index + 1;

      const auto& position_interface = state_interfaces_.at(pos_index);
      const auto& velocity_interface = state_interfaces_.at(vel_index);

      auto position_value = position_interface.get_optional();
      auto velocity_value = velocity_interface.get_optional();

      if (position_value.has_value()) {
        q_[robot_index](i) = position_value.value();
      } else {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Failed to get position value for joint %d of robot %zu", i, robot_index);
      }

      if (velocity_value.has_value()) {
        dq_[robot_index](i) = velocity_value.value();
      } else {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Failed to get velocity value for joint %d of robot %zu", i, robot_index);
      }
    }
  }
}

void MobileFr3DuoJointImpedanceExampleController::updateMobileBaseCommand(
    const rclcpp::Duration& period) {
  // Simple periodic forward/backward motion for the mobile base
  double cycle = std::floor(pow(
      -1.0, (elapsed_time_ - std::fmod(elapsed_time_, k_mobile_time_max_)) / k_mobile_time_max_));
  double v = cycle * k_mobile_v_max_ / 2.0 *
             (1.0 - std::cos(2.0 * M_PI / k_mobile_time_max_ * elapsed_time_));
  double v_x = std::cos(k_mobile_angle_) * v;
  double v_y = std::sin(k_mobile_angle_) * v;

  // Set command interfaces directly (vx, vy, vz, wx, wy, wz)
  if (!command_interfaces_[mobile_vx_index_].set_value(v_x)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set vx command");
  }
  if (!command_interfaces_[mobile_vy_index_].set_value(v_y)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set vy command");
  }
  if (!command_interfaces_[mobile_vz_index_].set_value(0.0)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set vz command");
  }
  if (!command_interfaces_[mobile_wx_index_].set_value(0.0)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set wx command");
  }
  if (!command_interfaces_[mobile_wy_index_].set_value(0.0)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set wy command");
  }
  if (!command_interfaces_[mobile_wz_index_].set_value(0.0)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Failed to set wz command");
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MobileFr3DuoJointImpedanceExampleController,
                       controller_interface::ControllerInterface)
