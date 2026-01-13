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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "franka_msgs/srv/self_collision.hpp"
#include "franka_selfcollision/self_collision_checker.hpp"

using namespace std::chrono_literals;

class CollisionCheckerNode : public rclcpp::Node {
 public:
  explicit CollisionCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("self_collision_service", options) {
    this->declare_parameter("security_margin", 0.045);
    this->declare_parameter("print_collisions", false);
  }

  void setup_collision_checker(const std::string& urdf_xml, const std::string& srdf_xml) {
    urdf_xml_ = urdf_xml;
    srdf_xml_ = srdf_xml;

    double security_margin = this->get_parameter("security_margin").as_double();
    print_collisions = this->get_parameter("print_collisions").as_bool();

    if (urdf_xml.empty() || srdf_xml.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameters 'robot_description' (URDF) or 'robot_description_semantic' (SRDF)"
                   "are empty.");
      throw std::runtime_error("Missing XML descriptions");
    }

    RCLCPP_INFO(this->get_logger(), "Loading robot model");

    // Initialize Collision checker
    try {
      collision_checker_ = std::make_shared<franka_selfcollision::SelfCollisionChecker>(
          urdf_xml, srdf_xml, security_margin);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load models: %s", e.what());
      throw;
    }

    std::vector<std::string> joint_order = collision_checker_->getModelJointNames();

    std::stringstream order_stream;
    order_stream << "Service expects lexicographically sorted Joint Order: [";
    for (size_t i = 1; i < joint_order.size(); ++i) {
      order_stream << joint_order[i] << (i < joint_order.size() - 1 ? ", " : "");
    }
    order_stream << "]";
    RCLCPP_INFO(this->get_logger(), "%s", order_stream.str().c_str());

    service_ = this->create_service<franka_msgs::srv::SelfCollision>(
        "check_self_collision",
        [this](const std::shared_ptr<franka_msgs::srv::SelfCollision::Request> request,
               std::shared_ptr<franka_msgs::srv::SelfCollision::Response> response) {
          this->check_collision_callback(request, response);
        });

    RCLCPP_INFO(this->get_logger(), "Self-Collision Service Ready. (Margin: %.3f m)",
                security_margin);
  }

  void check_collision_callback(
      const std::shared_ptr<franka_msgs::srv::SelfCollision::Request> request,
      std::shared_ptr<franka_msgs::srv::SelfCollision::Response> response) {
    std::vector<double> joint_configurations(request->joint_configuration.begin(),
                                             request->joint_configuration.end());
    response->collision =
        collision_checker_->checkCollision(joint_configurations, print_collisions);
  }

 private:
  std::string urdf_xml_;
  std::string srdf_xml_;
  bool print_collisions = false;
  std::shared_ptr<franka_selfcollision::SelfCollisionChecker> collision_checker_;
  rclcpp::Service<franka_msgs::srv::SelfCollision>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string package_share = ament_index_cpp::get_package_share_directory("franka_selfcollision");
  std::string params_file = package_share + "/config/self_collision_service.yaml";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_file});

  auto node = std::make_shared<CollisionCheckerNode>(options);

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/controller_manager");
  param_client->wait_for_service();
  auto future = param_client->get_parameters({"robot_description", "robot_description_semantic"});

  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto results = future.get();
    std::string urdf = results[0].as_string();
    std::string srdf = results[1].as_string();

    node->setup_collision_checker(urdf, srdf);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to fetch parameters");
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}