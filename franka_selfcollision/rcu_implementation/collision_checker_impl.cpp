#include "collision/collision_checker_impl.hpp"

#include <coal/collision_data.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Dense>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include "fmt/format.h"

namespace {

constexpr double kSecurityMargin = 0.045;  // in meters. Be careful with this value to avoid
                                           // unwanted or non-existent collisions.
constexpr bool kEnableContact = true;      // calculate contact point and normal
                                           // instead of just collision.
constexpr bool kStopAtFirstCollision = true;

std::string readFileToString(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    robot_control_unit::RCULogger::logError("CollisionChecker: error opening file: {}", filename);
    return "";
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  file.close();

  return oss.str();
}

void logCollisionInformation(const bool is_colliding,
                             const pinocchio::GeometryModel& pinocchio_geometry_model,
                             const pinocchio::GeometryData& pinocchio_geometry_data) {
  static constexpr size_t kCycleThreshold{1000};
  static std::atomic<size_t> cycle_count{0};

  cycle_count++;

  if (cycle_count.load() >= kCycleThreshold) {
    if (is_colliding) {
      const auto& cp =
          pinocchio_geometry_model.collisionPairs[pinocchio_geometry_data.collisionPairIndex];

      robot_control_unit::RCULogger::logError(
          "CollisionChecker: {} and {} under the distance security margin",
          pinocchio_geometry_model.geometryObjects[cp.first].name,
          pinocchio_geometry_model.geometryObjects[cp.second].name);
    }
    cycle_count.store(0);
  }
}

}  // namespace

namespace robot_control_unit {

PinocchioCollisionCheckerImpl::PinocchioCollisionCheckerImpl(const std::string& urdf_file,
                                                             const std::string& srdf_file,
                                                             const std::string& collision_meshes) {
  try {
    urdf_string_ = readFileToString(urdf_file);

    // Build the model and collision geometries
    pinocchio::urdf::buildModel(urdf_file, pinocchio_model_);
    pinocchio::urdf::buildGeom(pinocchio_model_, urdf_file, pinocchio::COLLISION,
                               pinocchio_geometry_model_, collision_meshes);

    // Parse URDF
    auto urdf_model = urdf::parseURDF(urdf_string_);
    if (!urdf_model) {
      throw std::runtime_error("CollisionChecker: failed to parse URDF file");
    }

    std::string last_joint_name = "joint" + std::to_string(pinocchio_model_.nq + 1);
    urdf::JointConstSharedPtr fixed_joint = urdf_model->getJoint(last_joint_name);

    // Extract the origin (rotation and offset)
    const urdf::Pose& offset = fixed_joint->parent_to_joint_origin_transform;
    last_link_to_flange_quaternion_ = Eigen::Quaterniond(offset.rotation.w, offset.rotation.x,
                                                         offset.rotation.y, offset.rotation.z);
    last_link_to_flange_position_ = Eigen::Vector3d(static_cast<double>(offset.position.x),
                                                    static_cast<double>(offset.position.y),
                                                    static_cast<double>(offset.position.z));
    // Add end-effector spheres
    end_effector_spheres_.reserve(kMaxNumEESpheres);
    for (size_t i = 0; i < kMaxNumEESpheres; ++i) {
      end_effector_spheres_.emplace_back("end_effector_sphere_" + std::to_string(i));
      auto& sphere = end_effector_spheres_.back();
      pinocchio_geometry_model_.addGeometryObject(createPinocchioObject(sphere, last_joint_name));
    }

    // Add all collision pairs and remove unwanted ones
    pinocchio_geometry_model_.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(pinocchio_model_, pinocchio_geometry_model_, srdf_file);

  } catch (const std::exception& exception) {
    std::string error_message =
        exception.what() +
        fmt::format(
            " CollisionChecker: could not load the arm URDF {}, SRDF {}, or collision meshes in {}",
            urdf_file, srdf_file, collision_meshes);
    RCULogger::logError(error_message);

    // Delay the kill in order for the log to be written
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    throw std::runtime_error(error_message);
  }
  removeCollisionPairsOutsideSRDF();

  // Initialize data storage
  pinocchio_data_ = pinocchio::Data(pinocchio_model_);
  pinocchio_geometry_data_ = pinocchio::GeometryData(pinocchio_geometry_model_);
}

void PinocchioCollisionCheckerImpl::removeCollisionPairsOutsideSRDF() {
  // TODO(wink_ma): check if that still works once accelerometer frames are there
  const std::array<std::string, 2> flange_names = {"link6_0", "link7_0"};
  std::vector<pinocchio::GeomIndex> flange_indices;
  for (auto& flange_name : flange_names) {
    if (!pinocchio_geometry_model_.existGeometryName(flange_name)) {
      const std::string message =
          "CollisionCheckerImpl: flange name " + flange_name + " does not exist in geometry model";
      RCULogger::logError(message);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      throw std::runtime_error(message);
    }
    const pinocchio::GeomIndex flange_index = pinocchio_geometry_model_.getGeometryId(flange_name);
    flange_indices.push_back(flange_index);
  }

  std::vector<pinocchio::GeomIndex> sphere_indices;
  pinocchio::GeomIndex sphere_index;
  for (auto& sphere : end_effector_spheres_) {
    sphere_index = pinocchio_geometry_model_.getGeometryId(sphere.name);
    for (auto& other_sphere_index : sphere_indices) {
      const pinocchio::CollisionPair sphere_pair(sphere_index, other_sphere_index);
      pinocchio_geometry_model_.removeCollisionPair(sphere_pair);
    }
    for (auto& flange_index : flange_indices) {
      const pinocchio::CollisionPair flange_pair(sphere_index, flange_index);
      pinocchio_geometry_model_.removeCollisionPair(flange_pair);
    }
    sphere_indices.push_back(sphere_index);
  }
  return;
}

bool PinocchioCollisionCheckerImpl::computeCollision(
    pinocchio::GeometryData& geometry_data,
    const pinocchio::PairIndex collision_pair_index) {
  auto& collision_request = geometry_data.collisionRequests[collision_pair_index];
  collision_request.enable_contact = kEnableContact;
  collision_request.security_margin = kSecurityMargin;

  return pinocchio::computeCollision(pinocchio_geometry_model_, geometry_data, collision_pair_index,
                                     collision_request);
}

bool PinocchioCollisionCheckerImpl::computeCollisions(pinocchio::GeometryData& geometry_data) {
  const auto& geometry_model = pinocchio_geometry_model_;

  for (std::size_t collision_pair_index = 0;
       collision_pair_index < geometry_model.collisionPairs.size(); ++collision_pair_index) {
    const auto& collision_pair = geometry_model.collisionPairs[collision_pair_index];

    if (geometry_data.activeCollisionPairs[collision_pair_index] &&
        !(geometry_model.geometryObjects[collision_pair.first].disableCollision ||
          geometry_model.geometryObjects[collision_pair.second].disableCollision)) {
      bool is_colliding = computeCollision(geometry_data, collision_pair_index);

      if (is_colliding) {
        geometry_data.collisionPairIndex = collision_pair_index;  // first pair to be in collision
        return true;
      }
    }
  }

  return false;
}

CollisionResult PinocchioCollisionCheckerImpl::isSelfColliding(
    const std::array<double, kNumJoints>& joint_configuration) {
  CollisionResult result{};
  const Eigen::Map<const Eigen::VectorXd> joint_configuration_vector(joint_configuration.data(),
                                                                     joint_configuration.size());

  pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, joint_configuration_vector);
  pinocchio::updateGeometryPlacements(pinocchio_model_, pinocchio_data_, pinocchio_geometry_model_,
                                      pinocchio_geometry_data_);
  bool is_colliding = computeCollisions(pinocchio_geometry_data_);

  result.collision = is_colliding;
  if (is_colliding) {
    const auto& collision_result =
        pinocchio_geometry_data_.collisionResults[pinocchio_geometry_data_.collisionPairIndex];
    result.distance = collision_result.distance_lower_bound;
    result.normal = collision_result.normal;
    result.points = collision_result.nearest_points;
  }

  logCollisionInformation(is_colliding, pinocchio_geometry_model_, pinocchio_geometry_data_);

  return result;
}

void PinocchioCollisionCheckerImpl::setEndEffectorVolumes(const SetEndEffectorVolumes& command) {
  for (size_t i = 0; i < kMaxNumEESpheres; i++) {
    auto& sphere = end_effector_spheres_[i];

    // Set the sphere properties
    sphere.setProperties(command, i);

    // Create sphere shape and placement (relative to the last link)
    std::shared_ptr<coal::Sphere> sphere_shape;
    pinocchio::SE3 sphere_placement;
    std::tie(sphere_shape, sphere_placement) = mapToObjectAndPlacement(sphere);

    // Update the geometry model
    auto geom_index = pinocchio_geometry_model_.getGeometryId(sphere.name);
    pinocchio_geometry_model_.geometryObjects[geom_index].geometry = sphere_shape;
    pinocchio_geometry_model_.geometryObjects[geom_index].placement = sphere_placement;
  }
}

pinocchio::GeometryObject PinocchioCollisionCheckerImpl::createPinocchioObject(
    const EndEffectorSphere& sphere,
    const std::string& last_joint_name) {
  // Get frame and parent joint info
  pinocchio::FrameIndex ee_frame_id = pinocchio_model_.getFrameId(last_joint_name);
  pinocchio::JointIndex parent_joint_id = pinocchio_model_.frames[ee_frame_id].parentJoint;

  // Create sphere shape and placement (relative to the last link)
  std::shared_ptr<coal::Sphere> sphere_shape;
  pinocchio::SE3 sphere_placement;
  std::tie(sphere_shape, sphere_placement) = mapToObjectAndPlacement(sphere);

  return pinocchio::GeometryObject(sphere.name, parent_joint_id, sphere_shape, sphere_placement);
}

std::pair<std::shared_ptr<coal::Sphere>, pinocchio::SE3>
PinocchioCollisionCheckerImpl::mapToObjectAndPlacement(const EndEffectorSphere& sphere) {
  std::shared_ptr<coal::Sphere> sphere_shape;
  pinocchio::SE3 sphere_placement;

  if (sphere.active) {
    sphere_shape = std::make_shared<coal::Sphere>(sphere.radius);
    sphere_placement = pinocchio::SE3(last_link_to_flange_quaternion_,
                                      last_link_to_flange_position_ + sphere.center);
  } else {
    sphere_shape = std::make_shared<coal::Sphere>(0.0);
    sphere_placement =
        pinocchio::SE3(last_link_to_flange_quaternion_, last_link_to_flange_position_);
  }

  return std::make_pair(sphere_shape, sphere_placement);
}

}  // namespace robot_control_unit