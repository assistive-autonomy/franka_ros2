#include <rclcpp/rclcpp.hpp>
#include "franka_msgs/srv/self_collision.hpp" 

// Pinocchio headers
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

using namespace std::chrono_literals;
 
class CollisionCheckerNode : public rclcpp::Node
{
public:
    CollisionCheckerNode() : Node("self_collision_node")
    {
        this->declare_parameter("urdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo.urdf");
        this->declare_parameter("srdf_path", "/ros2_ws/src/franka_description/urdfs/fr3_duo.srdf");
        this->declare_parameter("meshes_path", "/ros2_ws/src/franka_description/");

        auto urdf_path = this->get_parameter("urdf_path").as_string();
        auto srdf_path = this->get_parameter("srdf_path").as_string();
        auto meshes_path = this->get_parameter("meshes_path").as_string();

        if (urdf_path.empty() || srdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide 'urdf_path' and 'srdf_path' parameters.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading URDF: %s", urdf_path.c_str());

        // Initialize Pinocchio Models
        try {
            pinocchio::urdf::buildModel(urdf_path, model_);
            pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_);

            geom_model_.addAllCollisionPairs();
            pinocchio::srdf::removeCollisionPairs(model_, geom_model_, srdf_path);

            //remove gripper joints for now
            std::vector<pinocchio::JointIndex> locked_joints;
            for (pinocchio::JointIndex i = 1; i < model_.joints.size(); ++i) {
                std::string joint_name = model_.names[i];
                if (joint_name.find("finger") != std::string::npos) {
                    locked_joints.push_back(i);
                }
            }
            Eigen::VectorXd q_ref = pinocchio::neutral(model_);
            pinocchio::Model reduced_model;
            pinocchio::GeometryModel reduced_geom_model;
            pinocchio::buildReducedModel(model_, geom_model_, locked_joints, q_ref, reduced_model, reduced_geom_model);
            model_ = reduced_model;
            geom_model_ = reduced_geom_model;

            //continue with the setup
            data_ = std::make_shared<pinocchio::Data>(model_);
            geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);

            //check that everything is setup correctly
            RCLCPP_INFO(this->get_logger(), "--- Pinocchio Joint List ---");
            for (size_t i = 0; i < model_.names.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Joint Index %zu: %s", i, model_.names[i].c_str());
            }
            RCLCPP_INFO(this->get_logger(), "----------------------------");

            RCLCPP_INFO(this->get_logger(), "Pinocchio model loaded successfully with %d joints.", model_.nq);

        } catch (const std::exception & e){
            RCLCPP_ERROR(this->get_logger(), "Failed to load models: %s", e.what());
        }

        service_ = this->create_service<franka_msgs::srv::SelfCollision>(
            "check_self_collision", 
            [this](const std::shared_ptr<franka_msgs::srv::SelfCollision::Request> request,
                std::shared_ptr<franka_msgs::srv::SelfCollision::Response> response) {
                this->check_collision_callback(request, response);
            }
        );
    }

    void check_collision_callback(const std::shared_ptr<franka_msgs::srv::SelfCollision::Request> request,
                                  std::shared_ptr<franka_msgs::srv::SelfCollision::Response> response)
    {
        // Safety check
        if (request->joint_configuration.size() != (size_t)model_.nq){
            RCLCPP_WARN(this->get_logger(), "Dimension mismatch: Req %zu vs Model %d ", 
                        request->joint_configuration.size(), model_.nq);
            response->collision = false;
            return;
        }

        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
            request->joint_configuration.data(), 
            request->joint_configuration.size()
        );

        //pinocchio::updateGeometryPlacements(model_, *data_, geom_model_, *geom_data_, q);
        
        auto start = std::chrono::high_resolution_clock::now();

        bool is_colliding = pinocchio::computeCollisions(model_, *data_, geom_model_, *geom_data_, q, true);

        auto end = std::chrono::high_resolution_clock::now();

        double elapsed = std::chrono::duration<double, std::milli>(end-start).count();
        RCLCPP_INFO(this->get_logger(), "Math only %.4f ms", elapsed);
        
        if (is_colliding){
            RCLCPP_INFO(this->get_logger(), "Collision detected!");
        }

        bool collision_found = false;

        for(size_t k = 0; k < geom_model_.collisionPairs.size(); ++k)
        {
            const pinocchio::CollisionPair & cp = geom_model_.collisionPairs[k];
            const hpp::fcl::CollisionResult & cr = geom_data_->collisionResults[k];

            if(cr.isCollision())
            {
                collision_found = true;
                
                std::string name1 = geom_model_.geometryObjects[cp.first].name;
                std::string name2 = geom_model_.geometryObjects[cp.second].name;

                RCLCPP_INFO(this->get_logger(), "Collision detected: Pair %zu (%s, %s)", 
                            k, name1.c_str(), name2.c_str());
            }
        }
            

        response->collision = collision_found;
    }

private:
    pinocchio::Model model_;
    pinocchio::GeometryModel geom_model_;
    std::shared_ptr<pinocchio::Data> data_;
    std::shared_ptr<pinocchio::GeometryData> geom_data_;

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