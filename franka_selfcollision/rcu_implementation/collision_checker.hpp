#pragma once

#include "collision_checker_interface.hpp"
#include "collision_types.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

namespace robot_control_unit {

using Configuration = std::array<double, kNumJoints>;

class CollisionCheckerImpl;

class CollisionChecker : public CollisionCheckerInterface {
 public:
  /**
   * Constructs the CollisionChecker.
   *
   * @param collision_thread_affinity The CPU core to run the collision thread on (optional). If not
   *   set, it will use the same core as the caller thread.
   */
  CollisionChecker(std::optional<unsigned int> collision_thread_affinity = std::nullopt);
  ~CollisionChecker() override;

  /**
   * Initializes the collision checker with the URDF and SRDF file paths.
   * This function should be called only once and deferred until the URDF
   * and SRDF files are loaded.
   *
   * @param[in] urdf_file_path Path to the URDF file.
   * @param[in] srdf_file_path Path to the SRDF file.
   * @param[in] collision_meshes_path Path to the collision meshes.
   */
  void initialize(const std::string& urdf_file_path,
                  const std::string& srdf_file_path,
                  const std::string& collision_meshes_path) override;

  /**
   * Initializes the collision checker with a custom implementation.
   *
   * @param[in] impl Implementation of the collision checker.
   */
  void initialize(std::unique_ptr<CollisionCheckerImpl> impl);

  /**
   * Returns the last valid joint positions.
   *
   * @return Array of joint configurations.
   */
  [[nodiscard]] Configuration getLastValidJointConfiguration() const override;

  /**
   * Returns the result from the last self collision check.
   *
   * @return CollisionResult
   */
  [[nodiscard]] CollisionResult getLastSelfCollisionResult() const;

  /**
   * Checks for robot self collision in a thread by using the Pinocchio collision library.
   * - Returns the result from the *previous* check (if ready), otherwise the last result.
   * - Schedules a new check in the background.
   *
   * @param[in] joint_configuration Array of joint configurations.
   *
   * @return Boolean indicating if self collision is detected or not.
   */
  bool isSelfColliding(const double (&joint_configuration)[kNumJoints]) override;

  /**
   * Checks for robot self collision in a thread by using the Pinocchio collision library.
   * - Returns the result from the *previous* check (if ready), otherwise the last result.
   * - Schedules a new check in the background.
   *
   * @param[in] joint_configuration Array of joint configurations.
   *
   * @return Boolean indicating if self collision is detected or not.
   */
  bool isSelfColliding(const Configuration& joint_configuration);

  /**
   * Sets the end effector volumes for collision checking in a thread.
   *
   * @param[in] command Command to set the end effector volumes.
   */
  void setEndEffectorVolumes(const SetEndEffectorVolumes& command) override;

 private:
  /**
   * Sets the thread process priorities to non-realtime and move them to a different core (if
   * configured).
   */
  void configureThreads();

  /**
   * Retrieve the result from the previous collision check and return if missed the deadline.
   *
   * @return Boolean indicating if the previous collision check result is ready.
   */
  [[nodiscard]] bool retrieveLastCollisionResult();

  /**
   * Save the last collision result.
   *
   * @param[in] collision_result Collision result to save.
   */
  void saveLastCollisionResult(const CollisionResult& collision_result);

  /**
   * Mark the last joint configuration as valid.
   */
  void saveLastValidJointConfiguration();

  /**
   * Submit a task to the worker thread and return a future.
   *
   * @param[in] f Function to execute.
   * @param[in] args Arguments to the function.
   *
   * @return Future of the result.
   */
  template <typename F,
            typename... Args,
            typename R = std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>>
  [[nodiscard]] std::future<R> submitTask(F&& f, Args&&... args);

  /**
   * Detach a task to the worker thread.
   *
   * @param[in] f Function to execute.
   * @param[in] args Arguments to the function.
   */
  template <typename F, typename... Args>
  void detachTask(F&& f, Args&&... args);

  /**
   * Detach a task to the worker thread.
   *
   * @param[in] task Task to execute.
   */
  template <typename F>
  void detachTask(F&& task);

  /**
   * Create worker threads.
   *
   * @param[in] num_threads Number of threads to create.
   */
  void createThreads(const std::size_t num_threads);

  /**
   * Worker thread function.
   *
   * @param[in] stoken Stop token for the thread.
   */
  void workerThreadFunction(std::stop_token stoken) noexcept;

  // Pointer to the implementation
  std::unique_ptr<CollisionCheckerImpl> impl_;

  // Thread and synchronization
  std::optional<unsigned int> collision_thread_affinity_;
  std::queue<std::function<void()>> scheduled_tasks_;
  mutable std::mutex create_threads_mutex_;
  mutable std::mutex tasks_mutex_;
  mutable std::mutex last_collision_mutex_;
  mutable std::mutex last_joint_configuration_mutex_;
  std::condition_variable_any task_available_cv_;
  std::atomic<bool> initialized_{false};
  std::vector<std::jthread> worker_threads_;
  // Here order of declaration matters: workers must be after task_available_cv_,
  // so the first to be destroyed and joined. Otherwise unit tests will hang.

  // For pipelined retrieval of the last collision result
  std::future<CollisionResult> last_collision_future_;

  // For storing the last collision result
  CollisionResult last_collision_result_;

  // For storing the last valid joint configurations
  Configuration last_joint_configuration_;
  Configuration last_valid_joint_configuration_;
};

}  // namespace robot_control_unit