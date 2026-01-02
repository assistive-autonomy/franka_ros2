#include <rclcpp/rclcpp.hpp>
#include "franka_msgs/srv/self_collision.hpp" 
#include "franka_selfcollision/self_collision_checker.hpp"


using namespace std::chrono_literals;
 
class CollisionCheckerNode : public rclcpp::Node
{
public:
    CollisionCheckerNode() : Node("self_collision_service")
    {
        this->declare_parameter("robot_description", "");
        this->declare_parameter("robot_description_semantic", "");
        this->declare_parameter("security_margin", 0.045);
        this->declare_parameter("print_collisions", false);

        auto urdf_xml = this->get_parameter("robot_description").as_string();
        auto srdf_xml = this->get_parameter("robot_description_semantic").as_string();
        double security_margin = this->get_parameter("security_margin").as_double();
        print_collisions = this->get_parameter("print_collisions").as_bool();

        if (urdf_xml.empty() || srdf_xml.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameters 'robot_description' (URDF) or 'robot_description_semantic' (SRDF) are empty.");
            throw std::runtime_error("Missing XML descriptions");
        }

        RCLCPP_INFO(this->get_logger(), "Loading robot model");

        // Initialize Collision checker
        try {
            collision_checker_ = std::make_shared<franka_selfcollision::SelfCollisionChecker>(
                urdf_xml,
                srdf_xml,
                security_margin
            );
        } catch (const std::exception & e){
            RCLCPP_ERROR(this->get_logger(), "Failed to load models: %s", e.what());
            throw;
        }

        service_ = this->create_service<franka_msgs::srv::SelfCollision>(
            "check_self_collision", 
            [this](const std::shared_ptr<franka_msgs::srv::SelfCollision::Request> request,
                std::shared_ptr<franka_msgs::srv::SelfCollision::Response> response) {
                this->check_collision_callback(request, response);
            }
        );

        RCLCPP_INFO(this->get_logger(), "Self-Collision Service Ready. (Margin: %.3f m)", security_margin);
    }

    void check_collision_callback(const std::shared_ptr<franka_msgs::srv::SelfCollision::Request> request,
                                  std::shared_ptr<franka_msgs::srv::SelfCollision::Response> response)
    {
        
        std::vector<double> joint_configurations(request->joint_configuration.begin(), 
                                      request->joint_configuration.end());
        response->collision = collision_checker_->checkCollision(joint_configurations, print_collisions);
    }

private:
    bool print_collisions = false;
    std::shared_ptr<franka_selfcollision::SelfCollisionChecker> collision_checker_;
    rclcpp::Service<franka_msgs::srv::SelfCollision>::SharedPtr service_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionCheckerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}