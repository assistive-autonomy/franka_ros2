#ifndef FRANKA_SELFCOLLISION__SELF_COLLISION_CHECKER_HPP_
#define FRANKA_SELFCOLLISION__SELF_COLLISION_CHECKER_HPP_

#include <string>
#include <vector>
#include <sstream>

#ifndef PINOCCHIO_WITH_HPP_FCL
  #define PINOCCHIO_WITH_HPP_FCL
#endif

//Pinocchio header
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

namespace franka_selfcollision
{
class SelfCollisionChecker
{
public:
    /**
     * @brief Constructor loads the URDF/SRDF and builds the Pinocchio models.
     * @param urdf_xml XML of the robots URDF file.
     * @param srdf_xml XML of the robots SRDF file (for disabling allowable collisions).
     * @param security_margin Safety buffer in meters (default 0.045).
     */
    SelfCollisionChecker(const std::string& urdf_xml,
                         const std::string& srdf_xml,
                         double security_margin = 0.045);

    /**
     * @brief Checks if the given joint configuration results in a self-collision.
     * @param joint_configuration Vector of joint positions.
     * @param print_collisions If true, prints the names of colliding links to stdout.
     * @return true if collision detected, false otherwise.
     */
    bool checkCollision(const std::vector<double>& joint_configuration, bool print_collisions = false);

    /**
     * @brief Eigen overload for faster checking (zero-copy).
     */
    bool checkCollisions(const Eigen::VectorXd& q, bool print_collisions = false);

    int getDoF() const { return model_.nq; }

private:
    pinocchio::Model model_;
    pinocchio::GeometryModel geom_model_;
    std::shared_ptr<pinocchio::Data> data_;
    std::shared_ptr<pinocchio::GeometryData> geom_data_;

};

} // namespace franka_selfcollision

#endif //FRANKA_SELFCOLLISION__SELF_COLLISION_CHECKER_HPP_