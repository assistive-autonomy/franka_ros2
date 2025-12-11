#pragma once

#include <urdf_model/model.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

#include <string>
#include <vector>

struct CollisionResult {
    bool collision;
    std::string colliding_pair_name; 
};

class PinocchioCollisionCheckerImpl {
public:
    PinocchioCollisionCheckerImpl(const std::string& urdf_file,
                                  const std::string& srdf_file,
                                  double security_margin = 0.0);

    CollisionResult isSelfColliding(const Eigen::VectorXd& q);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel geom_model_;
    pinocchio::GeometryData geom_data_;

    double security_margin_;

    bool computeCollisions();
    bool computeCollisionPair(const pinocchio::PairIndex& pair_id);
};