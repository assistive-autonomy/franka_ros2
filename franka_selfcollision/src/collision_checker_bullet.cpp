#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <chrono>
#include <string>
#include <fstream>
#include <sstream>

// MoveIt Headers
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/collision_detection_bullet/collision_env_bullet.hpp>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.hpp>
#include <moveit/collision_detection/collision_tools.hpp>
#include <moveit/robot_state/conversions.hpp>

// Parsers
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

// Service Header
#include "franka_msgs/srv/self_collision_bullet.hpp"

class BulletCollisionNode : public rclcpp::Node
{
public:
    BulletCollisionNode(): Node("self_collision_node_bullet")
    {

        this->declare_parameter("urdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo.urdf");
        this->declare_parameter("srdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo_arms.srdf");

        auto urdf_path = this->get_parameter("urdf_path").as_string();
        auto srdf_path = this->get_parameter("srdf_path").as_string();

        if (urdf_path.empty() || srdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide 'urdf_path' and 'srdf_path'.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading URDF from: %s", urdf_path.c_str());

        std::string urdf_xml = readFile(urdf_path);
        std::string srdf_xml = readFile(srdf_path);

        if (urdf_xml.empty() || srdf_xml.empty()) {
            throw std::runtime_error("Could not read URDF/SRDF files from disk.");
        }

        auto urdf_model = urdf::parseURDF(urdf_xml);
        auto srdf_model = std::make_shared<srdf::Model>();
        srdf_model->initString(*urdf_model, srdf_xml);

        auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model);

        planning_scene_->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

        RCLCPP_INFO(this->get_logger(), "Active Collision Detector: %s", planning_scene_->getCollisionDetectorName().c_str());

        variable_names_ = robot_model->getVariableNames();

        service_ = this->create_service<franka_msgs::srv::SelfCollisionBullet>(
            "check_self_collision", 
            [this](const std::shared_ptr<franka_msgs::srv::SelfCollisionBullet::Request> request,
                std::shared_ptr<franka_msgs::srv::SelfCollisionBullet::Response> response) {
                this->check_collision_callback(request, response);
            }
        );

        RCLCPP_INFO(this->get_logger(), "Bullet Collision Service Ready.");
    }

    std::string readFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) return "";
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    void check_collision_callback(const std::shared_ptr<franka_msgs::srv::SelfCollisionBullet::Request> request,
                                  std::shared_ptr<franka_msgs::srv::SelfCollisionBullet::Response> response)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();

        // 1. Get sizes
        size_t request_size = request->joint_configuration.size();
        size_t model_size = variable_names_.size();

        // 2. Check for mismatch
        if (request_size != model_size) {
            RCLCPP_ERROR(this->get_logger(), "WRONG SIZES DETECTED!");
            RCLCPP_ERROR(this->get_logger(), "Input Vector Size: %zu", request_size);
            RCLCPP_ERROR(this->get_logger(), "Model Expects Size: %zu", model_size);
            
            RCLCPP_ERROR(this->get_logger(), "--- Joints Expected by RobotModel ---");
            for (size_t i = 0; i < model_size; ++i) {
                // Print Index and Name
                RCLCPP_ERROR(this->get_logger(), "Index [%zu]: %s", i, variable_names_[i].c_str());
            }
            RCLCPP_ERROR(this->get_logger(), "---------------------------------------");

            // Stop here to avoid crashing
            response->collision = false; 
            return; 
        }

        current_state.setVariablePositions(request->joint_configuration.data());
        current_state.updateCollisionBodyTransforms();

        collision_detection::CollisionResult c_res;
        collision_detection::CollisionRequest c_req;
        c_req.group_name = "";
        c_req.contacts = true;
        c_req.max_contacts = 10;
        c_req.max_contacts_per_pair = 1;
        c_req.verbose = false; 

        planning_scene_->checkSelfCollision(c_req, c_res);

        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        RCLCPP_INFO(this->get_logger(), "Math only %.4f ms", elapsed);

        if (c_res.collision) {
            for (const auto& contact_pair : c_res.contacts) {
                const std::string& link_A = contact_pair.first.first;
                const std::string& link_B = contact_pair.first.second;
                
                RCLCPP_WARN(this->get_logger(), "Collision detected: (%s <--> %s)", 
                            link_A.c_str(), link_B.c_str());
            }
        }

        response->collision = c_res.collision;
    }

private:
    planning_scene::PlanningScenePtr planning_scene_;
    std::vector<std::string> variable_names_;
    rclcpp::Service<franka_msgs::srv::SelfCollisionBullet>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BulletCollisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}