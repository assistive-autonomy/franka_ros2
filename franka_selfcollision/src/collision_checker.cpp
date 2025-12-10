#include "collision/collision_checker.hpp"
#include "collision/collision_checker_impl.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <utility>

namespace {

constexpr size_t kNumThreads = 1;

void logMissedDeadlines(const bool result_available) {
  static constexpr size_t kCycleThreshold{10000};
  static std::atomic<size_t> cycle_count{0};
  static std::atomic<size_t> missed_deadlines{0};

  cycle_count++;
  if (!result_available) {
    missed_deadlines++;
  }

  if (cycle_count.load() >= kCycleThreshold) {
    if (missed_deadlines.load() > 0) {
      robot_control_unit::RCULogger::logInfo("CollisionChecker: missed {} deadlines on {} cycles",
                                             missed_deadlines, kCycleThreshold);
    }
    missed_deadlines.store(0);
    cycle_count.store(0);
  }
}

}  // namespace

namespace robot_control_unit {

CollisionChecker::CollisionChecker(std::optional<unsigned int> collision_thread_affinity)
    : collision_thread_affinity_(collision_thread_affinity) {}

CollisionChecker::~CollisionChecker() = default;

void CollisionChecker::initialize(const std::string& urdf_file_path,
                                  const std::string& srdf_file_path,
                                  const std::string& collision_meshes_path) {
  initialize(std::make_unique<PinocchioCollisionCheckerImpl>(urdf_file_path, srdf_file_path,
                                                             collision_meshes_path));
}

void CollisionChecker::initialize(std::unique_ptr<CollisionCheckerImpl> impl) {
  // Use compare_exchange to check if we are initializing for the first time
  bool expected = false;
  if (!initialized_.compare_exchange_strong(expected, true)) {
    RCULogger::logError("CollisionChecker is already running. Initialization skipped.");
    return;
  }

  impl_ = std::move(impl);
  createThreads(kNumThreads);
}

Configuration CollisionChecker::getLastValidJointConfiguration() const {
  if (!initialized_.load()) {
    throw std::runtime_error("CollisionChecker is not initialized");
  }

  std::scoped_lock<std::mutex> lock(last_joint_configuration_mutex_);
  return last_valid_joint_configuration_;
}

CollisionResult CollisionChecker::getLastSelfCollisionResult() const {
  if (!initialized_.load()) {
    throw std::runtime_error("CollisionChecker is not initialized");
  }

  std::scoped_lock<std::mutex> lock(last_collision_mutex_);
  return last_collision_result_;
}

bool CollisionChecker::isSelfColliding(const double (&joint_configuration)[kNumJoints]) {
  Configuration joint_configuration_array = std::to_array(joint_configuration);
  return isSelfColliding(joint_configuration_array);
}

bool CollisionChecker::isSelfColliding(const Configuration& joint_configuration) {
  if (!initialized_.load()) {
    throw std::runtime_error("CollisionChecker is not initialized");
  }

  bool result_available = retrieveLastCollisionResult();
  logMissedDeadlines(result_available);

  if (!result_available) {
    return last_collision_result_.collision;
  }

  auto future =
      submitTask(&CollisionCheckerImpl::isSelfColliding, impl_.get(), joint_configuration);
  last_collision_future_ = std::move(future);  // save this future for next iteration's retrieval
  last_joint_configuration_ = joint_configuration;  // save joint configuration as a candidate for
                                                    // last_valid_joint_configuration_

  return last_collision_result_.collision;  // return the previous iteration's collision result
}

void CollisionChecker::setEndEffectorVolumes(const SetEndEffectorVolumes& command) {
  if (!initialized_.load()) {
    throw std::runtime_error("CollisionChecker is not initialized");
  }

  detachTask(&CollisionCheckerImpl::setEndEffectorVolumes, impl_.get(), command);
}

bool CollisionChecker::retrieveLastCollisionResult() {
  CollisionResult last_collision_result{};  // default to no collision at first iteration
  bool result_available = true;             // assume the result is available at first iteration

  if (last_collision_future_.valid()) {  // check if a task was scheduled

    if (last_collision_future_.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready) {  // check if the result is available
      result_available = true;

      try {
        last_collision_result = last_collision_future_.get();
      } catch (const std::exception& exception) {
        RCULogger::logError("CollisionChecker: exception while getting last collision result: {}",
                            exception.what());
      }
      saveLastCollisionResult(last_collision_result);
      if (!last_collision_result_.collision) {
        saveLastValidJointConfiguration();
      }

    } else {
      result_available = false;
    }
  }

  return result_available;
}

void CollisionChecker::saveLastCollisionResult(const CollisionResult& collision_result) {
  std::scoped_lock<std::mutex> lock(last_collision_mutex_);
  last_collision_result_ = collision_result;
}

void CollisionChecker::saveLastValidJointConfiguration() {
  std::scoped_lock<std::mutex> lock(last_joint_configuration_mutex_);
  last_valid_joint_configuration_ = last_joint_configuration_;
}

template <typename F, typename... Args, typename R>
std::future<R> CollisionChecker::submitTask(F&& f, Args&&... args) {
  // Create a promise and a future for the result
  auto promise = std::make_shared<std::promise<R>>();
  std::future<R> future = promise->get_future();

  // Create a task from the function and arguments
  auto task = [promise, function = std::decay_t<F>(std::forward<F>(f)),
               tuple = std::make_tuple(std::decay_t<Args>(std::forward<Args>(args))...)]() mutable {
    try {
      if constexpr (std::is_void_v<R>) {
        std::apply(function, tuple);
        promise->set_value();
      } else {
        promise->set_value(std::apply(function, tuple));
      }
    } catch (...) {
      promise->set_exception(std::current_exception());  // propagate the exception to the future
    }
  };

  // Push the task to the queue
  detachTask(std::move(task));
  return future;
}

template <typename F, typename... Args>
void CollisionChecker::detachTask(F&& f, Args&&... args) {
  // Create a task from the function and arguments
  auto task = [function = std::decay_t<F>(std::forward<F>(f)),
               tuple = std::make_tuple(std::decay_t<Args>(std::forward<Args>(args))...)]() mutable {
    std::apply(function, tuple);
  };

  // Push the task to the queue
  detachTask(std::move(task));
}

template <typename F>
void CollisionChecker::detachTask(F&& task) {
  {
    std::scoped_lock lock(tasks_mutex_);
    scheduled_tasks_.push(std::forward<F>(task));
  }
  task_available_cv_.notify_one();
}

void CollisionChecker::createThreads(const std::size_t num_threads) {
  std::scoped_lock lock(create_threads_mutex_);

  worker_threads_.resize(num_threads);

  for (auto& thread : worker_threads_) {
    thread = std::jthread([this](std::stop_token stoken) { workerThreadFunction(stoken); });
  }

  configureThreads();
}

void CollisionChecker::configureThreads() {
  sched_param params;
  params.sched_priority = 0;

  cpu_set_t cpuset;
  for (auto& thread : worker_threads_) {
    // Set thread scheduling policy, priority and on which CP core to run
    pthread_setschedparam(thread.native_handle(), SCHED_OTHER, &params);

    if (collision_thread_affinity_) {
      CPU_ZERO(&cpuset);
      CPU_SET(*collision_thread_affinity_, &cpuset);
      pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset);
    }
  }
}

void CollisionChecker::workerThreadFunction(std::stop_token stoken) noexcept {
  while (true) {
    std::function<void()> task;

    {
      std::unique_lock<std::mutex> lock(tasks_mutex_);

      // Wait until either stop is requested or scheduled_tasks_ is not empty
      task_available_cv_.wait(lock, stoken, [this] { return !scheduled_tasks_.empty(); });

      // If stop has been requested, exit the loop
      if (stoken.stop_requested()) {
        return;
      }

      // Pop a task from the queue
      task = std::move(scheduled_tasks_.front());
      scheduled_tasks_.pop();
    }

    // Execute the task outside the lock
    try {
      task();
    } catch (const std::exception& exception) {
      RCULogger::logError("CollisionChecker: exception while executing task: {}", exception.what());
    }
  }
}

}  // namespace robot_control_unit