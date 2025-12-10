#pragma once

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <vector>

namespace robot_control_unit {

using Configuration = std::array<double, kNumJoints>;

/**
 * @brief Interface for collision checking implementations.
 *
 * This interface defines the methods that a collision checker implementation must provide.
 */
class CollisionCheckerImpl {
 public:
  virtual ~CollisionCheckerImpl() = default;
  /**
   * @brief Check if the robot is self-colliding.
   *
   * @param joint_configuration The joint configuration to check for self-collisions.
   * @return CollisionResult The result of the collision check.
   */
  virtual CollisionResult isSelfColliding(const Configuration& joint_configuration) = 0;
  /**
   * @brief Set the end effector volumes for the robot.
   *
   * @param command The command to set the end effector volumes.
   */
  virtual void setEndEffectorVolumes(const SetEndEffectorVolumes& command) = 0;
};

class PinocchioCollisionCheckerImpl : public CollisionCheckerImpl {
 public:
  PinocchioCollisionCheckerImpl() = delete;

  /**
   * @brief Construct a new PinocchioCollisionCheckerImpl object.
   *
   * @param urdf_file The URDF file of the robot.
   * @param srdf_file The SRDF file of the robot.
   * @param collision_meshes The collision meshes of the robot.
   */
  PinocchioCollisionCheckerImpl(const std::string& urdf_file,
                                const std::string& srdf_file,
                                const std::string& collision_meshes);

  ~PinocchioCollisionCheckerImpl() override = default;

  /**
   * @brief Check if the robot is self-colliding.
   *
   * @param joint_configuration The joint configuration to check for self-collisions.
   * @return CollisionResult The result of the collision check.
   */
  CollisionResult isSelfColliding(const Configuration& joint_configuration) override;

  /**
   * @brief Set the end effector volumes for the robot.
   *
   * @param command The command to set the end effector volumes.
   */
  void setEndEffectorVolumes(const SetEndEffectorVolumes& command) override;

 private:
  /**
   * @brief Check if the robot is self-colliding.
   *
   * @param geometry_data The geometry data of the robot.
   * @param pair_id The pair of collision objects to check for collisions.
   * @return bool true if the objects are colliding, false otherwise.
   */
  bool computeCollision(pinocchio::GeometryData& geometry_data, const pinocchio::PairIndex pair_id);

  /**
   * @brief Check if the robot is self-colliding.
   *
   * @param geometry_data The geometry data of the robot.
   * @return bool true if the robot is self-colliding, false otherwise.
   */
  bool computeCollisions(pinocchio::GeometryData& geometry_data);

  /**
   * @brief Remove collision pairs that are not contained in the SRDF.
   */
  void removeCollisionPairsOutsideSRDF();

  /**
   * @brief Create a Pinocchio object from an end effector sphere.
   *
   * @param sphere The end effector sphere.
   * @param last_joint_name The name of the last joint to attach the sphere to.
   *
   * @return pinocchio::GeometryObject The Pinocchio object.
   */
  pinocchio::GeometryObject createPinocchioObject(const EndEffectorSphere& sphere,
                                                  const std::string& last_joint_name);

  /**
   * @brief Map an end effector sphere to a sphere object and placement.
   *
   * @param sphere The end effector sphere.
   *
   * @return std::pair<std::shared_ptr<coal::Sphere>, pinocchio::SE3> The sphere object and
   * placement.
   */
  std::pair<std::shared_ptr<coal::Sphere>, pinocchio::SE3> mapToObjectAndPlacement(
      const EndEffectorSphere& sphere);

  std::string urdf_string_;
  pinocchio::Model pinocchio_model_;
  pinocchio::GeometryModel pinocchio_geometry_model_;
  pinocchio::Data pinocchio_data_;
  pinocchio::GeometryData pinocchio_geometry_data_;
  std::vector<EndEffectorSphere> end_effector_spheres_;
  Eigen::Quaterniond last_link_to_flange_quaternion_;
  Eigen::Vector3d last_link_to_flange_position_;
};

}  // namespace robot_control_unit