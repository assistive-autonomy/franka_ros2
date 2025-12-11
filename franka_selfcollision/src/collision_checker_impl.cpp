#include "collision/collision_checker_impl.hpp"

#include <urdf_model/model.h>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

PinocchioCollisionCheckerImpl::PinocchioCollisionCheckerImpl(
    const std::string& urdf_file,
    const std::string& srdf_file,
    double security_margin) 
    : security_margin_(security_margin)
{
    pinocchio::urdf::buildModel(urdf_file, model_);
    pinocchio::urdf::buildGeom(model_, urdf_file, pinocchio::COLLISION, geom_model_);

    geom_model_.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model_, geom_model_, srdf_file);

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

    data_ = pinocchio::Data(model_);
    geom_data_ = pinocchio::GeometryData(geom_model_);

    for (auto& request : geom_data_.collisionRequests) {
        request.security_margin = security_margin_;
        request.enable_contact = false; 
    }
}

bool PinocchioCollisionCheckerImpl::computeCollisionPair(const pinocchio::PairIndex& pair_id) {
    return pinocchio::computeCollision(geom_model_, geom_data_, pair_id);
}

bool PinocchioCollisionCheckerImpl::computeCollisions() {
    for (size_t i = 0; i < geom_model_.collisionPairs.size(); ++i) {

        if (geom_data_.activeCollisionPairs[i]) {
            
            if (computeCollisionPair(i)) {
                geom_data_.collisionPairIndex = i;
                return true; 
            }
        }
    }
    return false;
}

CollisionResult PinocchioCollisionCheckerImpl::isSelfColliding(const Eigen::VectorXd& q) {
    CollisionResult result;
    result.collision = false;

    pinocchio::forwardKinematics(model_, data_, q);

    pinocchio::updateGeometryPlacements(model_, data_, geom_model_, geom_data_);

    if (computeCollisions()) {
        result.collision = true;
        
        const auto& pair = geom_model_.collisionPairs[geom_data_.collisionPairIndex];
        std::string n1 = geom_model_.geometryObjects[pair.first].name;
        std::string n2 = geom_model_.geometryObjects[pair.second].name;
        result.colliding_pair_name = n1 + " <-> " + n2;
    }

    return result;
}