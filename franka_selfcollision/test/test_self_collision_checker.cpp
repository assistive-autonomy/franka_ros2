#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "franka_selfcollision/self_collision_checker.hpp"

std::string readFileToString(const std::string& filename) {
  auto file = std::ifstream(filename);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return "";
  }

  auto oss = std::ostringstream();
  oss << file.rdbuf();
  file.close();

  return oss.str();
}

class SelfCollisionCheckerTest : public ::testing::Test {
 protected:
  static constexpr double kSecurityMargin = 0.001;
  static constexpr size_t kNumJoints = 14;

  void SetUp() override {
    try {
      std::string test_dir = TEST_DIR;

      std::string urdf_path = test_dir + "/fr3_duo.urdf";
      std::string urdf_xml = readFileToString(urdf_path);

      std::string srdf_path = test_dir + "/fr3_duo.srdf";
      std::string srdf_xml = readFileToString(srdf_path);

      checker_ = std::make_unique<franka_selfcollision::SelfCollisionChecker>(urdf_xml, srdf_xml,
                                                                              kSecurityMargin);
    } catch (const std::exception& e) {
      FAIL() << "Setup failed" << e.what();
    }
  }

  std::unique_ptr<franka_selfcollision::SelfCollisionChecker> checker_;
};

TEST_F(SelfCollisionCheckerTest, ThrowsOnIncorrectInputDimensions) {
  std::vector<double> input_too_small(SelfCollisionCheckerTest::kNumJoints - 1, 0.0);
  std::vector<double> input_too_big(SelfCollisionCheckerTest::kNumJoints + 1, 0.0);

  EXPECT_THROW({ checker_->checkCollision(input_too_small, false); }, std::exception);

  EXPECT_THROW({ checker_->checkCollision(input_too_big, false); }, std::exception);
}

TEST_F(SelfCollisionCheckerTest, ReturnsFalseForSafeConfigurations) {
  std::vector<double> home_config(SelfCollisionCheckerTest::kNumJoints, 0.0);

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