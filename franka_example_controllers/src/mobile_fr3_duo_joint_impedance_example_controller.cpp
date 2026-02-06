#include <franka_example_controllers/mobile_fr3_duo_joint_impedance_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cmath>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

static constexpr size_t BASE_STATE_IFS = 4 * 2;  // base joints: pos + vel
static constexpr size_t ARM_STATE_IFS = 7 * 2;   // arm joints: pos + vel
static constexpr size_t BASE_CMD_IFS = 6;        // vx vy vz wx wy wz
static constexpr size_t ARM_CMD_IFS = 7;

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (enable_mobile_base_) {
    config.names = {"vx/cartesian_velocity", "vy/cartesian_velocity", "vz/cartesian_velocity",
                    "wx/cartesian_velocity", "wy/cartesian_velocity", "wz/cartesian_velocity"};
  }

  for (const auto& prefix : arm_prefixes_) {
    if (prefix.empty()) {
      continue;  // Skip empty prefixes (e.g., for the mobile base)
    }
    for (int i = 1; i <= num_arm_joints; ++i) {
      config.names.push_back(prefix + "_fr3_joint" + std::to_string(i) + "/effort");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
MobileFr3DuoJointImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // base state
  for (int i = 0; i < num_base_joints; ++i) {
    config.names.push_back(robot_types_[0] + "_joint_" + std::to_string(i) + "/position");
    config.names.push_back(robot_types_[0] + "_joint_" + std::to_string(i) + "/velocity");
  }

  // left arm
  for (int i = 1; i <= num_arm_joints; ++i) {
    config.names.push_back(arm_prefixes_[1] + "_" + robot_types_[1] + "_joint" + std::to_string(i) +
                           "/position");
    config.names.push_back(arm_prefixes_[1] + "_" + robot_types_[1] + "_joint" + std::to_string(i) +
                           "/velocity");
  }

  // right arm
  for (int i = 1; i <= num_arm_joints; ++i) {
    config.names.push_back(arm_prefixes_[2] + "_" + robot_types_[2] + "_joint" + std::to_string(i) +
                           "/position");
    config.names.push_back(arm_prefixes_[2] + "_" + robot_types_[2] + "_joint" + std::to_string(i) +
                           "/velocity");
  }

  return config;
}

controller_interface::return_type MobileFr3DuoJointImpedanceExampleController::update(
    const rclcpp::Time&,
    const rclcpp::Duration& period) {
  updateJointStates();
  elapsed_time_ += period.seconds();

  // ---- arms impedance ----
  for (size_t arm = 0; arm < 2; ++arm) {
    Vector7d q_goal = initial_q_[arm];

    double delta = M_PI / 8.0 * (1.0 - std::cos(M_PI / 2.5 * elapsed_time_));
    q_goal(3) += delta;
    q_goal(4) += delta;

    constexpr double kAlpha = 0.99;
    dq_filtered_[arm] = (1.0 - kAlpha) * dq_filtered_[arm] + kAlpha * dq_[arm];

    Vector7d tau =
        k_gains_.cwiseProduct(q_goal - q_[arm]) + d_gains_.cwiseProduct(-dq_filtered_[arm]);

    size_t cmd_offset = BASE_CMD_IFS + arm * ARM_CMD_IFS;
    for (size_t j = 0; j < 7; ++j) {
      if (!command_interfaces_[cmd_offset + j].set_value(tau(j))) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Failed to set torque for arm %zu joint %zu", arm, j);
      }
    }
  }

  // ---- mobile base motion ----
  if (enable_mobile_base_) {
    updateMobileBaseCommand(period);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_init() {
  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<std::string>>("arm_prefixes", {});
    auto_declare<std::vector<std::string>>("robot_types", {});
    auto_declare<bool>("enable_mobile_base", true);
  } catch (...) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State&) {
  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  arm_prefixes_ = get_node()->get_parameter("arm_prefixes").as_string_array();
  enable_mobile_base_ = get_node()->get_parameter("enable_mobile_base").as_bool();
  robot_types_ = get_node()->get_parameter("robot_types").as_string_array();

  if (k.size() != 7 || d.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains and d_gains must be size 7");
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < num_arm_joints; ++i) {
    k_gains_(i) = k[i];
    d_gains_(i) = d[i];
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_activate(
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

CallbackReturn MobileFr3DuoJointImpedanceExampleController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return CallbackReturn::SUCCESS;
}

void MobileFr3DuoJointImpedanceExampleController::updateJointStates() {
  for (size_t arm = 0; arm < 2; ++arm) {
    for (auto i = 0; i < num_arm_joints; ++i) {
      size_t pos = BASE_STATE_IFS + arm * ARM_STATE_IFS + i * 2;
      size_t vel = pos + 1;

      auto p = state_interfaces_[pos].get_optional();
      auto v = state_interfaces_[vel].get_optional();

      if (p && v) {
        q_[arm](i) = *p;
        dq_[arm](i) = *v;
      } else {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Missing state for arm %zu joint %zu", arm, i);
      }
    }
  }
}

void MobileFr3DuoJointImpedanceExampleController::updateMobileBaseCommand(const rclcpp::Duration&) {
  double cycle = std::floor(std::pow(
      -1.0, (elapsed_time_ - std::fmod(elapsed_time_, k_mobile_time_max_)) / k_mobile_time_max_));

  double v = cycle * k_mobile_v_max_ / 2.0 *
             (1.0 - std::cos(2.0 * M_PI / k_mobile_time_max_ * elapsed_time_));

  double v_x = std::cos(k_mobile_angle_) * v;
  double v_y = std::sin(k_mobile_angle_) * v;

  command_interfaces_[0].set_value(v_x);
  command_interfaces_[1].set_value(v_y);
  command_interfaces_[2].set_value(0.0);
  command_interfaces_[3].set_value(0.0);
  command_interfaces_[4].set_value(0.0);
  command_interfaces_[5].set_value(0.0);
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MobileFr3DuoJointImpedanceExampleController,
                       controller_interface::ControllerInterface)
