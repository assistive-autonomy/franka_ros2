#include <rclcpp/rclcpp.hpp>
#include "franka_msgs/srv/self_collision.hpp" 
#include "franka_selfcollision/collision_checker_impl.hpp" 

class CollisionCheckerNode : public rclcpp::Node
{
public:
    CollisionCheckerNode() : Node("self_collision_node_fast")
    {
        // Parameters
        this->declare_parameter("urdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo.urdf");
        this->declare_parameter("srdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo.srdf");
        this->declare_parameter("security_margin", 0.045);

        auto urdf_path = this->get_parameter("urdf_path").as_string();
        auto srdf_path = this->get_parameter("srdf_path").as_string();
        double security_margin = this->get_parameter("security_margin").as_double();

        checker_ = std::make_unique<PinocchioCollisionCheckerImpl>(urdf_path, srdf_path, security_margin);

        RCLCPP_INFO(this->get_logger(), "Fast Collision Checker Ready.");

        service_ = this->create_service<franka_msgs::srv::SelfCollision>(
            "check_self_collision", 
            [this](const std::shared_ptr<franka_msgs::srv::SelfCollision::Request> req,
                   std::shared_ptr<franka_msgs::srv::SelfCollision::Response> res) {
                
                Eigen::Map<const Eigen::VectorXd> q(req->joint_configuration.data(), 
                                                    req->joint_configuration.size());

                auto start = std::chrono::high_resolution_clock::now();

                CollisionResult result = checker_->isSelfColliding(q);

                auto end = std::chrono::high_resolution_clock::now();

                double elapsed = std::chrono::duration<double, std::milli>(end-start).count();
                RCLCPP_INFO(this->get_logger(), "Math only %.4f ms", elapsed);
                
                res->collision = result.collision;
                
                if (result.collision) {
                     RCLCPP_WARN(this->get_logger(), "Collision: %s", result.colliding_pair_name.c_str());
                }
            }
        );
    }

private:
    std::unique_ptr<PinocchioCollisionCheckerImpl> checker_;
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