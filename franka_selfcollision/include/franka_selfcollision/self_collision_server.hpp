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

#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "franka_msgs/action/self_collision.hpp"
#include "franka_selfcollision/self_collision_checker.hpp"

namespace franka_selfcollision {

class SelfCollisionServer : public rclcpp::Node {
 public:
  using CheckSelfCollision = franka_msgs::action::SelfCollision;
  using GoalHandleCheck = rclcpp_action::ServerGoalHandle<CheckSelfCollision>;

  explicit SelfCollisionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void setup_self_collision(const std::string& urdf_xml, const std::string& srdf_xml);

 private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const CheckSelfCollision::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCheck> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleCheck> goal_handle);

  void execute(const std::shared_ptr<GoalHandleCheck> goal_handle);

  rclcpp_action::Server<CheckSelfCollision>::SharedPtr action_server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::mutex mutex_;
  std::vector<double> current_joint_positions_;

  std::unordered_map<std::string, size_t> joint_map_;

  std::shared_ptr<franka_selfcollision::SelfCollisionChecker> collision_checker_;

  bool print_collisions_{false};
  double security_margin_{0.01};
};

}  // namespace franka_selfcollision
