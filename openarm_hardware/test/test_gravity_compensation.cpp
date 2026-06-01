// Copyright 2026 OpenArm contributors.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "openarm_hardware/gravity_compensation.hpp"

TEST(GravityCompensatorTest, InitAndComputeLeftArm) {
  const char* urdf_path_env = std::getenv("OPENARM_GRAVITY_TEST_URDF");
  if (urdf_path_env == nullptr || urdf_path_env[0] == '\0') {
    GTEST_SKIP() << "Set OPENARM_GRAVITY_TEST_URDF to a bimanual URDF path";
  }

  openarm_hardware::GravityCompensator comp;
  ASSERT_TRUE(comp.init(urdf_path_env, "openarm_body_link0", "openarm_left_hand"));

  std::array<double, openarm_hardware::GravityCompensator::kArmDof> q{};
  std::array<double, openarm_hardware::GravityCompensator::kArmDof> tau{};
  ASSERT_TRUE(comp.compute(q, tau));

  double sum = 0.0;
  for (double t : tau) {
    sum += std::abs(t);
  }
  EXPECT_GT(sum, 0.0);
}
