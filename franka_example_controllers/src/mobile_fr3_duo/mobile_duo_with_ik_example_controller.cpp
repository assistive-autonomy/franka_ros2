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

#include <franka_example_controllers/mobile_fr3_duo/mobile_duo_with_ik_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

static constexpr size_t kBaseStateInterfaces = 4;
static constexpr size_t kArmStateInterfaces = 7 * 2;
static constexpr size_t kBaseCommandInterfaces = 4;
static constexpr size_t kArmCommandInterfaces = 7;

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleControllerWithIK::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names = {robot_types_[0] + "_joint_0/position", robot_types_[0] + "_joint_1/velocity",
                  robot_types_[0] + "_joint_2/position", robot_types_[0] + "_joint_3/velocity"};

  for (int index = 0; index < arm_prefixes_.size(); ++index) {
    for (int i = 1; i <= num_arm_joints; ++i) {
      config.names.push_back(arm_prefixes_[index] + "_" + robot_types_[index + 1] + "_joint" +
                             std::to_string(i) + "/effort");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleControllerWithIK::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 0; i < num_base_joints; ++i) {
    if (i % 2 == 0) {
      config.names.push_back(robot_types_[0] + "_joint_" + std::to_string(i) + "/position");
    } else {
      config.names.push_back(robot_types_[0] + "_joint_" + std::to_string(i) + "/velocity");
    }
  }

  for (int arm = 0; arm < 2; ++arm) {
    for (int i = 1; i <= num_arm_joints; ++i) {
      std::string prefix =
          arm_prefixes_[arm] + "_" + robot_types_[arm + 1] + "_joint" + std::to_string(i);
      config.names.push_back(prefix + "/position");
      config.names.push_back(prefix + "/velocity");
    }
  }

  return config;
}

controller_interface::return_type MobileFr3DuoJointImpedanceExampleControllerWithIK::update(
    const rclcpp::Time&,
    const rclcpp::Duration& period) {
  updateJointStates();
  elapsed_time_ += period.seconds();

  for (size_t arm = 0; arm < 2; ++arm) {
    Vector7d q_goal = initial_q_[arm];

    double delta = M_PI / 8.0 * (1.0 - std::cos(M_PI / 2.5 * elapsed_time_));
    q_goal(3) += delta;
    q_goal(4) += delta;

    constexpr double kAlpha = 0.99;
    dq_filtered_[arm] = (1.0 - kAlpha) * dq_filtered_[arm] + kAlpha * dq_[arm];

    Vector7d tau =
        k_gains_.cwiseProduct(q_goal - q_[arm]) + d_gains_.cwiseProduct(-dq_filtered_[arm]);

    size_t cmd_offset = kBaseCommandInterfaces + arm * kArmCommandInterfaces;
    for (size_t j = 0; j < 7; ++j) {
      if (!command_interfaces_[cmd_offset + j].set_value(tau(j))) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Failed to set torque for arm %zu joint %zu", arm, j);
      }
    }
  }

  updateMobileBaseCommand(period);

  return controller_interface::return_type::OK;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleControllerWithIK::on_init() {
  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<std::string>>("robot_prefixes", {});
    auto_declare<std::vector<std::string>>("robot_types", {});
    auto_declare<double>("vx_amp", 0.1);
    auto_declare<double>("vy_amp", 0.0);
    auto_declare<double>("wz_amp", 0.0);
    auto_declare<double>("freq", 0.1);
    auto_declare<double>("wheel_radius", 0.1);
  } catch (...) {
    return CallbackReturn::ERROR;
  }

  wheel_positions_ << 0.3, -0.2, -0.3, 0.2;

  steering_angles_.setZero();
  wheel_velocities_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleControllerWithIK::on_configure(
    const rclcpp_lifecycle::State&) {
  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  robot_prefixes_ = get_node()->get_parameter("robot_prefixes").as_string_array();
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();
  vx_amp_ = get_node()->get_parameter("vx_amp").as_double();
  vy_amp_ = get_node()->get_parameter("vy_amp").as_double();
  wz_amp_ = get_node()->get_parameter("wz_amp").as_double();
  freq_ = get_node()->get_parameter("freq").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();

  auto arm_prefixes_begin = robot_prefixes_.begin() + 1;
  arm_prefixes_ = std::vector<std::string>(arm_prefixes_begin, arm_prefixes_begin + 2);

  if (k.size() != 7 || d.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains and d_gains must be size 7");
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_arm_joints; ++i) {
    k_gains_(i) = k[i];
    d_gains_(i) = d[i];
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleControllerWithIK::on_activate(
    const rclcpp_lifecycle::State&) {
  q_.resize(2, Vector7d::Zero());
  dq_.resize(2, Vector7d::Zero());
  dq_filtered_.resize(2, Vector7d::Zero());
  initial_q_.resize(2, Vector7d::Zero());

  updateJointStates();
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleControllerWithIK::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return CallbackReturn::SUCCESS;
}

void MobileFr3DuoJointImpedanceExampleControllerWithIK::updateJointStates() {
  for (int arm = 0; arm < 2; ++arm) {
    for (auto i = 0; i < num_arm_joints; ++i) {
      int pos = kBaseStateInterfaces + arm * kArmStateInterfaces + i * 2;
      int vel = pos + 1;

      auto p = state_interfaces_[pos].get_optional();
      auto v = state_interfaces_[vel].get_optional();

      if (p && v) {
        q_[arm](i) = *p;
        dq_[arm](i) = *v;
      } else {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Missing state for arm %d joint %d", arm, i);
      }
    }
  }
}

void MobileFr3DuoJointImpedanceExampleControllerWithIK::updateMobileBaseCommand(
    const rclcpp::Duration&) {
  constexpr double k_period = 10.0;
  constexpr double k_mobile_v_max_ = 0.2;
  constexpr double k_mobile_angle_ = 0.0;

  double cycle =
      std::floor(std::pow(-1.0, (elapsed_time_ - std::fmod(elapsed_time_, k_period)) / k_period));
  double v =
      cycle * k_mobile_v_max_ / 2.0 * (1.0 - std::cos(2.0 * M_PI / k_period * elapsed_time_));
  double v_x = std::cos(k_mobile_angle_) * v;
  double v_y = std::sin(k_mobile_angle_) * v;
  double wz = wz_amp_ * std::sin(2.0 * M_PI * freq_ * elapsed_time_);

  computeSwerveIK(v_x, v_y, wz);

  if (!command_interfaces_[0].set_value(commands_[0].steering_angle)) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to set steering angle for wheel 0: %f",
                commands_[0].steering_angle);
  }
  if (!command_interfaces_[1].set_value(commands_[0].wheel_velocity)) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to set wheel velocity for wheel 0: %f",
                commands_[0].wheel_velocity);
  }
  if (!command_interfaces_[2].set_value(commands_[1].steering_angle)) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to set steering angle for wheel 1: %f",
                commands_[1].steering_angle);
  }
  if (!command_interfaces_[3].set_value(commands_[1].wheel_velocity)) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to set wheel velocity for wheel 1: %f",
                commands_[1].wheel_velocity);
  }
}

void MobileFr3DuoJointImpedanceExampleControllerWithIK::computeSwerveIK(double vx,
                                                                        double vy,
                                                                        double wz) {
  Eigen::Array2d x = wheel_positions_.col(0).array();
  Eigen::Array2d y = wheel_positions_.col(1).array();

  Eigen::Array2d vx_wheel = vx - wz * y;
  Eigen::Array2d vy_wheel = vy + wz * x;

  Eigen::Array2d speed = ((vx_wheel.square() + vy_wheel.square()).sqrt()) / wheel_radius_;
  Eigen::Array2d angle;
  for (int i = 0; i < kNumberOfWheels; ++i) {
    angle(i) = std::atan2(vy_wheel(i), vx_wheel(i));
  }

  for (int i = 0; i < kNumberOfWheels; ++i) {
    double prev_angle = steering_angles_(i);
    double prev_velocity = wheel_velocities_(i);
    double new_angle = angle(i);
    double new_speed = speed(i);

    double angle_diff = std::fabs(new_angle - prev_angle);
    if (angle_diff > M_PI / 2.0) {
      commands_[i].steering_angle = prev_angle;
      commands_[i].wheel_velocity = -new_speed;
    } else {
      commands_[i].steering_angle = new_angle;
      commands_[i].wheel_velocity = new_speed;
    }
    steering_angles_(i) = commands_[i].steering_angle;
    wheel_velocities_(i) = commands_[i].wheel_velocity;
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::MobileFr3DuoJointImpedanceExampleControllerWithIK,
    controller_interface::ControllerInterface)