#include "franka_selfcollision/self_collision_checker.hpp"

//Pinocchio headers
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

namespace franka_selfcollision
{

SelfCollisionChecker::SelfCollisionChecker(const std::string& urdf_xml,
                                           const std::string& srdf_xml,
                                           double security_margin)
{

    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);

    std::istringstream urdf_stream(urdf_xml);
    pinocchio::urdf::buildGeom(model_, urdf_stream, pinocchio::COLLISION, geom_model_);

    geom_model_.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairsFromXML(model_, geom_model_, srdf_xml);

    //Create Reduced Model for finger joints if they are there
    std::vector<pinocchio::JointIndex> locked_joints;
    for (pinocchio::JointIndex i = 1; i < model_.joints.size(); ++i) {
        std::string joint_name = model_.names[i];
        if (joint_name.find("finger") != std::string::npos) {
            locked_joints.push_back(i);
        }
    }

    if (!locked_joints.empty()) {
        Eigen::VectorXd q_ref = pinocchio::neutral(model_);
        pinocchio::Model reduced_model;
        pinocchio::GeometryModel reduced_geom_model;

        pinocchio::buildReducedModel(model_, geom_model_, locked_joints, q_ref, reduced_model, reduced_geom_model);

        model_ = reduced_model;
        geom_model_ = reduced_geom_model;
    }

    data_ = std::make_shared<pinocchio::Data>(model_);
    geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);

    //Apply security margin
    for (auto& collision_request : geom_data_->collisionRequests) {
        collision_request.security_margin = security_margin;
        collision_request.enable_contact = true;
    }

}

bool SelfCollisionChecker::checkCollision(const std::vector<double>& joint_configuration, bool print_collisions) {
    if (joint_configuration.size() != (size_t)model_.nq) {
        std::cerr << "[SelfCollisionChecker] Dimension mismatch: Input "
                  << joint_configuration.size() << " vs Model " << model_.nq << std::endl;
        return false;
    }

    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(joint_configuration.data(), joint_configuration.size());
    
    return checkCollisions(q, print_collisions);
}

bool SelfCollisionChecker::checkCollisions(const Eigen::VectorXd& q, bool print_collisions)  {
    bool stop_at_first = !print_collisions;

    bool collision_found = pinocchio::computeCollisions(model_, *data_, geom_model_, *geom_data_, q, stop_at_first);

    if (collision_found && print_collisions) {
        for (size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) { 
            const pinocchio::CollisionPair & cp = geom_model_.collisionPairs[k];
            const hpp::fcl::CollisionResult & cr = geom_data_->collisionResults[k];

            if (cr.isCollision()) {
                const std::string & name1 = geom_model_.geometryObjects[cp.first].name;
                const std::string & name2 = geom_model_.geometryObjects[cp.second].name;
                
                std::cout << "[SelfCollisionChecker] Collision: " << name1 << " <--> " << name2 << std::endl;
            }
        }
    }

    return collision_found;
}

} // namespace franka_selfcollision
