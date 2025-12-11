#include "collision/collision_types.hpp"

namespace robot_control_unit {

EndEffectorSphere::EndEffectorSphere(const std::string& name) : name(name) {}

EndEffectorSphere::EndEffectorSphere(const std::string& name,
                                     const Eigen::Vector3d& center,
                                     double radius,
                                     bool active)
    : name(name), center(center), radius(radius), active(active) {
  if (radius < 0.0) {
    throw std::invalid_argument(
        fmt::format("EndEffectorSphere {} radius must be non-negative.", name));
  }
}

void EndEffectorSphere::setProperties(const SetEndEffectorVolumes& command, size_t index) {
  if (index >= kMaxNumEESpheres) {
    throw std::invalid_argument(
        fmt::format("EndEffectorSphere index = {} exceeds the max number of spheres = {}.", index,
                    kMaxNumEESpheres));
  }
  setProperties(Eigen::Vector3d(command.ee_centers[index * 3], command.ee_centers[index * 3 + 1],
                                command.ee_centers[index * 3 + 2]),
                command.ee_radii[index], command.ee_active[index]);
}

void EndEffectorSphere::setProperties(const Eigen::Vector3d& center, double radius, bool active) {
  if (radius < 0.0) {
    RCULogger::logError("EndEffectorSphere {} radius must be non-negative.", name);
    this->center = Eigen::Vector3d::Zero();
    this->radius = 0.0;
    this->active = false;
  } else {
    this->center = center;
    this->radius = radius;
    this->active = active;
  }
}

}  // namespace robot_control_unit