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
#include "franka_msgs/srv/self_collision.hpp"

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

  for (const auto& arm_id : arm_ids_) {
    config.names.push_back(arm_id + "/collision_detected");
  }
  return config;

}

controller_interface::InterfaceConfiguration
SelfCollisionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& arm_id : arm_ids_) {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_id + "_joint" + std::to_string(i) + "/position");
    }
  }

  return config;
}

controller_interface::return_type SelfCollisionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  
  {
    std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      for (size_t i = 0; i < state_interfaces_.size(); ++i){
        current_joint_positions_[i] = state_interfaces_[i].get_optional<double>().value();
      }
    }
  }

  double command_value = collision_detected_.load();

  for(auto& interface : command_interfaces_) {
    if(!interface.set_value(command_value)){
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to write collision command");
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SelfCollisionController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("arm_ids", {"left_fr3v2", "right_fr3v2"});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SelfCollisionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_ids_ = get_node()->get_parameter("arm_ids").as_string_array();

  joint_names_.clear();
  for (const auto& arm_id : arm_ids_) {
    for (int i = 1; i <= num_joints; ++i) {
      joint_names_.push_back(arm_id + "_joint" + std::to_string(i));
    }
  }

  collision_client_ = get_node()->create_client<franka_msgs::srv::SelfCollision>("/check_self_collision");
  
  current_joint_positions_.resize(joint_names_.size(), 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "Configured Self Collision for %zu joints.", joint_names_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn SelfCollisionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  collision_detected_.store(0.0);
  worker_running_ = true;
  worker_thread_ = std::thread(&SelfCollisionController::workerLogic, this);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SelfCollisionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  worker_running_ = false;
  if (worker_thread_.joinable()){
    worker_thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

void SelfCollisionController::workerLogic() {
 auto request = std::make_shared<franka_msgs::srv::SelfCollision::Request>();

 while (worker_running_ && rclcpp::ok()){
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      size_t copy_size = std::min(current_joint_positions_.size(), request->joint_configuration.size());
      std::copy(current_joint_positions_.begin(), 
                current_joint_positions_.begin() + copy_size, 
                request->joint_configuration.begin());
    }
    if (collision_client_->service_is_ready()){
      auto future = collision_client_->async_send_request(request);
      if (future.wait_for(std::chrono::milliseconds(2)) == std::future_status::ready) {
        auto result = future.get();
        double val = result->collision ? 1.0 : 0.0;
        collision_detected_.store(val);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

 }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SelfCollisionController,
                       controller_interface::ControllerInterface)
