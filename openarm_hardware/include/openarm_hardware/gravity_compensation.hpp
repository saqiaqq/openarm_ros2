// Copyright 2026 OpenArm contributors.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <array>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace openarm_hardware {

/// KDL-based gravity torque feedforward for a single 7-DoF arm chain.
class GravityCompensator {
 public:
  static constexpr std::size_t kArmDof = 7;

  bool init(const std::string& urdf_path,
            const std::string& root_link,
            const std::string& tip_link,
            double gravity_z = -9.81) {
    ready_ = false;

    std::ifstream file(urdf_path);
    if (!file.is_open()) {
      return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    urdf::Model model;
    if (!model.initString(buffer.str())) {
      return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      return false;
    }

    if (!tree.getChain(root_link, tip_link, chain_)) {
      return false;
    }

    if (chain_.getNrOfJoints() != static_cast<unsigned int>(kArmDof)) {
      return false;
    }

    gravity_forces_.resize(kArmDof);
    solver_ = std::make_unique<KDL::ChainDynParam>(
      chain_, KDL::Vector(0.0, 0.0, gravity_z));
    ready_ = true;
    return true;
  }

  bool ready() const { return ready_; }

  bool compute(const std::array<double, kArmDof>& q,
               std::array<double, kArmDof>& tau_g) const {
    if (!ready_ || !solver_) {
      return false;
    }

    KDL::JntArray q_kdl(kArmDof);
    for (std::size_t i = 0; i < kArmDof; ++i) {
      q_kdl(i) = q[i];
    }

    if (solver_->JntToGravity(q_kdl, gravity_forces_) < 0) {
      return false;
    }

    for (std::size_t i = 0; i < kArmDof; ++i) {
      tau_g[i] = gravity_forces_(i);
    }
    return true;
  }

 private:
  KDL::Chain chain_;
  mutable KDL::JntArray gravity_forces_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
  bool ready_{false};
};

}  // namespace openarm_hardware
