#include <rclcpp/rclcpp.hpp>
#include "franka_msgs/srv/self_collision.hpp" 
#include "franka_selfcollision/self_collision_checker.hpp"


using namespace std::chrono_literals;
 
class CollisionCheckerNode : public rclcpp::Node
{
public:
    CollisionCheckerNode() : Node("self_collision_service")
    {
        this->declare_parameter("urdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo.urdf");
        this->declare_parameter("srdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo_arms.srdf");
        this->declare_parameter("security_margin", 0.045);
        this->declare_parameter("print_collisions", false);

        auto urdf_path = this->get_parameter("urdf_path").as_string();
        auto srdf_path = this->get_parameter("srdf_path").as_string();
        double security_margin = this->get_parameter("security_margin").as_double();
        print_collisions = this->get_parameter("print_collisions").as_bool();

        if (urdf_path.empty() || srdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide 'urdf_path' and 'srdf_path' parameters.");
            throw std::runtime_error("Invalid parameters");
        }

        RCLCPP_INFO(this->get_logger(), "Loading URDF: %s", urdf_path.c_str());

        // Initialize Collision checker
        try {
            collision_checker_ = std::make_shared<franka_selfcollision::SelfCollisionChecker>(
                urdf_path,
                srdf_path,
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