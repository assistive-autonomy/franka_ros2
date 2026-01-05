#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "franka_selfcollision/self_collision_checker.hpp"

std::string execCommand(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;

  auto pipe_deleter = [](FILE* f) {
    if (f)
      pclose(f);
  };
  std::unique_ptr<FILE, decltype(pipe_deleter)> pipe(popen(cmd, "r"), pipe_deleter);

  if (!pipe) {
    throw std::runtime_error("popen failed!");
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

class SelfCollisionCheckerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    try {
      std::string cmd_urdf =
          "xacro /ros2_ws/src/franka_description/robots/fr3_duo/fr3_duo.urdf.xacro";
      std::string urdf_xml = execCommand(cmd_urdf.c_str());

      std::string cmd_srdf =
          "xacro /ros2_ws/src/franka_description/robots/fr3_duo/fr3_duo.srdf.xacro";
      std::string srdf_xml = execCommand(cmd_srdf.c_str());

      checker_ =
          std::make_unique<franka_selfcollision::SelfCollisionChecker>(urdf_xml, srdf_xml, 0.001);
    } catch (const std::exception& e) {
      FAIL() << "Setup failed" << e.what();
    }
  }

  std::unique_ptr<franka_selfcollision::SelfCollisionChecker> checker_;
};

TEST_F(SelfCollisionCheckerTest, ThrowsOnIncorrectInputDimensions) {
  std::vector<double> input_too_small(13, 0.0);
  std::vector<double> input_too_big(15, 0.0);

  EXPECT_THROW({ checker_->checkCollision(input_too_small, false); }, std::exception);

  EXPECT_THROW({ checker_->checkCollision(input_too_big, false); }, std::exception);
}

TEST_F(SelfCollisionCheckerTest, ReturnsFalseForSafeConfigurations) {
  // Home Configuration
  std::vector<double> home_config(14, 0.0);

  // Arms up Configuration
  std::vector<double> safe_config = {
      0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785,  // Arm 1
      0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785   // Arm 2
  };

  EXPECT_FALSE(checker_->checkCollision(home_config, true)) << "Home config should be safe";
  EXPECT_FALSE(checker_->checkCollision(safe_config, true)) << "Arms up should be safe";
}

TEST_F(SelfCollisionCheckerTest, ReturnsTrueForCollidingConfigurations) {
  // Left arm in bottom plate
  std::vector<double> mount_collision = {
      0.0, 1.0, 0.0, -2.6, 0.0, 2.5, 0.0,  // Arm 1
      0.0, 0.0, 0.0, -1.6, 0.0, 2.5, 0.0,  // Arm 2
  };

  // Arms into another Configuration
  std::vector<double> dual_collision = {
      -0.45, 0.77, 0.12, -1.38, 0.0,   2.4,  0.52,  // Arm 1
      0.26,  0.57, 0.03, -1.44, -0.38, 2.62, 1.34   // Arm 2
  };

  EXPECT_TRUE(checker_->checkCollision(mount_collision, true))
      << "Left arm should collide into the mount";
  EXPECT_TRUE(checker_->checkCollision(dual_collision, true))
      << "Arms should collide with each other";
}