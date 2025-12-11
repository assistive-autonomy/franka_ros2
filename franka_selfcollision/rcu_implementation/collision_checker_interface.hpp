#pragma once

namespace robot_control_unit {

class CollisionCheckerInterface {
 public:
  virtual ~CollisionCheckerInterface() = default;

  /**
   * Initializes the collision checker with the URDF and SRDF file paths.
   * This function should be called only once and deferred until the URDF
   * and SRDF files are loaded.
   *
   * @param[in] urdf_file_path Path to the URDF file.
   * @param[in] srdf_file_path Path to the SRDF file.
   * @param[in] collision_meshes_path Path to the collision meshes.
   */
  virtual void initialize(const std::string& urdf_file_path,
                          const std::string& srdf_file_path,
                          const std::string& collision_meshes_path) = 0;

  /**
   * Returns the last valid joint positions.
   *
   * @return Array of joint configurations.
   */
  [[nodiscard]] virtual std::array<double, kNumJoints> getLastValidJointConfiguration() const = 0;

  /**
   * Checks for self collision.
   *
   * @param[in] joint_configuration Joint configuration.
   * @return True if self collision is detected, false otherwise.
   */
  virtual bool isSelfColliding(const double (&joint_configuration)[kNumJoints]) = 0;

  /**
   * Sets the end effector volumes.
   *
   * @param[in] command SetEndEffectorVolumes command.
   */
  virtual void setEndEffectorVolumes(const SetEndEffectorVolumes& command) = 0;
};

}  // namespace robot_control_unit