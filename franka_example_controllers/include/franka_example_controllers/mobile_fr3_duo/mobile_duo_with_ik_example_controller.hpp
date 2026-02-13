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

#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_mobile_duo_example_controllers {

/**
 * The mobile FR3 duo example with IK is meant to be used for Gazebo Sim and combines:
 * - Dual arm joint impedance control (like fr3_duo)
 * - Mobile base cartesian velocity control (with swerve drive IK to send joint commands to Gazebo)
 * This controller moves joint 4 and 5 of both arms in a compliant periodic movement
 * while controlling the mobile base velocity.
 */
class MobileFr3DuoJointImpedanceExampleControllerWithIK
    : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::vector<std::string> robot_types_;
  std::vector<std::string> robot_prefixes_;
  std::vector<std::string> arm_prefixes_;
  std::string robot_description_;
  const int num_arm_joints = 7;
  const int num_base_joints = 4;

  std::vector<Vector7d> q_;
  std::vector<Vector7d> initial_q_;
  std::vector<Vector7d> dq_;
  std::vector<Vector7d> dq_filtered_;

  Vector7d k_gains_;
  Vector7d d_gains_;
  double elapsed_time_{0.0};

  static constexpr int kNumberOfWheels = 2;
  Eigen::Matrix2d wheel_positions_;
  Eigen::Vector2d steering_angles_;
  Eigen::Vector2d wheel_velocities_;
  struct Command {
    double steering_angle;
    double wheel_velocity;
  };
  std::array<Command, kNumberOfWheels> commands_;

  double vx_amp_{0.1};
  double vy_amp_{0.0};
  double wz_amp_{0.0};
  double freq_{0.1};
  double wheel_radius_{0.1};

  double k_period_{10.0};
  double k_mobile_v_max_{0.2};
  double k_mobile_angle_{0.0};

  void updateJointStates();
  void updateMobileBaseCommand(const rclcpp::Duration& period);
  void computeSwerveIK(double vx, double vy, double wz);
};

}  // namespace franka_mobile_duo_example_controllers