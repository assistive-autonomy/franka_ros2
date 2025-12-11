#pragma once

#include <Eigen/Core>

#include <array>
#include <stdexcept>
#include <string>

namespace robot_control_unit {

constexpr unsigned long kMaxNumEESpheres = 5;

/**
 * @brief Struct to store the result of a collision check.
 *
 * The struct contains information about the collision, the distance between the objects, the normal
 * of the collision and the points of the collision.
 *
 * @param collision True if a collision was detected, false otherwise.
 * @param distance The distance between the objects.
 * @param normal The normal of the collision.
 * @param points The points of the collision.
 */
struct CollisionResult {
  bool collision{false};
  double distance{0.0};
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 2> points = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
};

/**
 * @brief Struct to store the properties of an end effector sphere.
 *
 * The struct contains the name of the sphere, the center, the radius and if the sphere is active.
 *
 * @param name The name of the sphere.
 * @param center The center of the sphere.
 * @param radius The radius of the sphere.
 * @param active True if the sphere is active, false otherwise.
 */
class EndEffectorSphere {
 public:
  EndEffectorSphere() = delete;

  /**
   * @brief Construct a new End Effector Sphere object.
   *
   * @param name The name of the sphere.
   */
  EndEffectorSphere(const std::string& name);

  /**
   * @brief Construct a new End Effector Sphere object.
   *
   * @param name The name of the sphere.
   * @param center The center of the sphere.
   * @param radius The radius of the sphere.
   * @param active True if the sphere is active, false otherwise.
   */
  EndEffectorSphere(const std::string& name,
                    const Eigen::Vector3d& center,
                    double radius,
                    bool active);

  /**
   * @brief Set the properties of the sphere.
   *
   * @param command The command to set the properties.
   * @param index The index of the sphere.
   */
  void setProperties(const SetEndEffectorVolumes& command, size_t index);

  /**
   * @brief Set the properties of the sphere.
   *
   * @param center The center of the sphere.
   * @param radius The radius of the sphere.
   * @param active True if the sphere is active, false otherwise.
   */
  void setProperties(const Eigen::Vector3d& center, double radius, bool active);

  std::string name;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
  bool active{false};
};

}  // namespace robot_control_unit