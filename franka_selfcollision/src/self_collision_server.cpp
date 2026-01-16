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

#include "franka_selfcollision/self_collision_server.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sstream>

using namespace std::chrono_literals;

namespace franka_selfcollision {

SelfCollisionServer::SelfCollisionServer(const rclcpp::NodeOptions& options)
    : Node("self_collision_server", options) {
  this->declare_parameter("security_margin", 0.045);
  this->declare_parameter("print_collisions", false);

  action_server_ = rclcpp_action::create_server<CheckSelfCollision>(
      this, "check_self_collision",
      std::bind(&SelfCollisionServer::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&SelfCollisionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SelfCollisionServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Action Server created. Waiting for parameter setup...");
}

void SelfCollisionServer::setup_self_collision(const std::string& urdf_xml,
                                               const std::string& srdf_xml) {
  security_margin_ = this->get_parameter("security_margin").as_double();
  print_collisions_ = this->get_parameter("print_collisions").as_bool();

  if (urdf_xml.empty() || srdf_xml.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameters 'robot_description' (URDF) or 'robot_description_semantic' (SRDF) "
                 "are empty.");
    throw std::runtime_error("Missing XML descriptions");
  }

  RCLCPP_INFO(this->get_logger(), "Loading robot model...");

  // Initialize Collision checker
  try {
    collision_checker_ = std::make_shared<franka_selfcollision::SelfCollisionChecker>(
        urdf_xml, srdf_xml, security_margin_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load models: %s", e.what());
    throw;
  }

  const std::vector<std::string>& model_joint_names = collision_checker_->getModelJointNames();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_map_.clear();
    size_t index_counter = 0;
    std::stringstream order_stream;
    order_stream << "Model Joint Order: [";

    for (const auto& name : model_joint_names) {
      if (name == "universe")
        continue;

      joint_map_[name] = index_counter;
      order_stream << name << (index_counter < model_joint_names.size() - 2 ? ", " : "");
      index_counter++;
    }
    order_stream << "]";
    RCLCPP_INFO(this->get_logger(), "%s", order_stream.str().c_str());

    current_joint_positions_.resize(joint_map_.size(), 0.0);
  }

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->joint_state_callback(msg);
      });

  RCLCPP_INFO(this->get_logger(), "Self-Collision Monitor & Action Server Active. (Margin: %.3f m)",
              security_margin_);
}

void SelfCollisionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    auto it = joint_map_.find(msg->name[i]);
    if (it != joint_map_.end()) {
      size_t idx = it->second;
      if (i < msg->position.size() && idx < current_joint_positions_.size()) {
        current_joint_positions_[idx] = msg->position[i];
      }
    }
  }
}

rclcpp_action::GoalResponse SelfCollisionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const CheckSelfCollision::Goal> goal) {
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SelfCollisionServer::handle_cancel(
    const std::shared_ptr<GoalHandleCheck> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel collision monitoring.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SelfCollisionServer::handle_accepted(const std::shared_ptr<GoalHandleCheck> goal_handle) {
  std::thread{std::bind(&SelfCollisionServer::execute, this, goal_handle)}.detach();
}

void SelfCollisionServer::execute(const std::shared_ptr<GoalHandleCheck> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Starting continuous collision monitoring loop...");

  rclcpp::Rate loop_rate(30);

  auto feedback = std::make_shared<CheckSelfCollision::Feedback>();
  auto result = std::make_shared<CheckSelfCollision::Result>();

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->success = true;
      result->message = "Monitoring stopped by user.";
      goal_handle->canceled(result);
      return;
    }

    std::vector<double> q_check;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      q_check = current_joint_positions_;
    }

    bool is_colliding = collision_checker_->checkCollision(q_check, print_collisions_);

    feedback->is_colliding = is_colliding;
    goal_handle->publish_feedback(feedback);

    if (is_colliding) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "⚠️  COLLISION DETECTED! Robot is in self-collision! ⚠️");
    }

    loop_rate.sleep();
  }
}

};  // namespace franka_selfcollision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string package_share = ament_index_cpp::get_package_share_directory("franka_selfcollision");
  std::string params_file = package_share + "/config/self_collision_server.yaml";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_file});

  auto node = std::make_shared<franka_selfcollision::SelfCollisionServer>(options);

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/controller_manager");
  param_client->wait_for_service();
  auto future = param_client->get_parameters({"robot_description", "robot_description_semantic"});

  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto results = future.get();
    std::string urdf = results[0].as_string();
    std::string srdf = results[1].as_string();

    node->setup_self_collision(urdf, srdf);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to fetch parameters");
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}