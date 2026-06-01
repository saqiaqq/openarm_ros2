// Copyright 2025 Reazon Holdings, Inc.
// Copyright 2025 Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/openarm_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

static const std::string& can_device_name = "can0";

OpenArmHW::OpenArmHW() = default;

hardware_interface::CallbackReturn OpenArmHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // read hardware parameters
  if (info_.hardware_parameters.find("can_interface") ==
      info_.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "No can_interface parameter found");
    return CallbackReturn::ERROR;
  }

  auto it = info_.hardware_parameters.find("prefix");
  if (it == info_.hardware_parameters.end()) {
    prefix_ = "";
  } else {
    prefix_ = it->second;
  }
  it = info_.hardware_parameters.find("disable_torque");
  if (it == info_.hardware_parameters.end()) {
    disable_torque_ = false;
  } else {
    disable_torque_ = it->second == "true";
  }

  gravity_comp_enabled_ = false;
  gravity_scale_ = 1.0;
  gravity_comp_.reset();
  it = info_.hardware_parameters.find("gravity_compensation");
  if (it != info_.hardware_parameters.end() && it->second == "true") {
    gravity_comp_enabled_ = true;
    auto scale_it = info_.hardware_parameters.find("gravity_scale");
    if (scale_it != info_.hardware_parameters.end()) {
      try {
        gravity_scale_ = std::stod(scale_it->second);
      } catch (const std::exception&) {
        gravity_scale_ = 1.0;
      }
    }

    const auto urdf_it = info_.hardware_parameters.find("gravity_urdf_path");
    const auto root_it = info_.hardware_parameters.find("gravity_root_link");
    const auto tip_it = info_.hardware_parameters.find("gravity_tip_link");
    if (urdf_it == info_.hardware_parameters.end() ||
        root_it == info_.hardware_parameters.end() ||
        tip_it == info_.hardware_parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "gravity_compensation=true but gravity_urdf_path/root/tip missing");
      return CallbackReturn::ERROR;
    }

    gravity_comp_ = std::make_unique<GravityCompensator>();
    if (!gravity_comp_->init(urdf_it->second, root_it->second, tip_it->second)) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "Failed to init gravity compensation (urdf=%s root=%s tip=%s)",
                   urdf_it->second.c_str(), root_it->second.c_str(),
                   tip_it->second.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Gravity compensation enabled (scale=%.3f tip=%s)",
                gravity_scale_, tip_it->second.c_str());
  }

  // temp CANFD
  canbus_ = std::make_unique<CANBus>(info_.hardware_parameters.at("can_interface"),
                                     CAN_MODE_FD);
  motor_control_ = std::make_unique<MotorControl>(*canbus_);

  if (USING_GRIPPER) {
    motor_types.emplace_back(DM_Motor_Type::DM4310);
    can_device_ids.emplace_back(0x08);
    can_master_ids.emplace_back(0x18);
    ++curr_dof;
  }

  motors_.resize(curr_dof);
  for (size_t i = 0; i < curr_dof; ++i) {
    motors_[i] = std::make_unique<Motor>(motor_types[i], can_device_ids[i],
                                         can_master_ids[i]);
    motor_control_->addMotor(*motors_[i]);
  }

  pos_states_.resize(curr_dof, 0.0);
  pos_commands_.resize(curr_dof, 0.0);
  vel_states_.resize(curr_dof, 0.0);
  vel_commands_.resize(curr_dof, 0.0);
  tau_states_.resize(curr_dof, 0.0);
  tau_ff_commands_.resize(curr_dof, 0.0);
  gripper_speed_commands_.resize(curr_dof, 1.0);
  gripper_force_commands_.resize(curr_dof, 0.0);
  refresh_motors();
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  // zero position or calibrate to pose
  // for (std::size_t i = 0; i < curr_dof; ++i)
  // {
  //   motor_control_->set_zero_position(*motors_[i]);
  // }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < curr_dof; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &tau_states_[i]));
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Exporting state interface for joint %s",
                info_.joints[i].name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < curr_dof; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &tau_ff_commands_[i]));
  }
  if (USING_GRIPPER) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[GRIPPER_INDEX].name, "gripper_speed",
        &gripper_speed_commands_[GRIPPER_INDEX]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[GRIPPER_INDEX].name, "gripper_force",
        &gripper_force_commands_[GRIPPER_INDEX]));
  }

  return command_interfaces;
}

void OpenArmHW::refresh_motors() {
  for (const auto& motor : motors_) {
    motor_control_->controlMIT(*motor, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
}

double OpenArmHW::armGravityTorque(size_t joint_index) const {
  if (!gravity_comp_enabled_ || !gravity_comp_ || !gravity_comp_->ready() ||
      joint_index >= ARM_DOF) {
    return 0.0;
  }

  std::array<double, GravityCompensator::kArmDof> q{};
  std::array<double, GravityCompensator::kArmDof> tau_g{};
  for (size_t i = 0; i < ARM_DOF; ++i) {
    q[i] = pos_states_[i];
  }
  if (!gravity_comp_->compute(q, tau_g)) {
    return 0.0;
  }
  return gravity_scale_ * tau_g[joint_index];
}

hardware_interface::CallbackReturn OpenArmHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  bool zeroed = false;
  for (const auto& motor : motors_) {
    motor_control_->enable(*motor);
  }

  while (!zeroed) {
    bool all_zero = true;
    for (std::size_t m = 0; m < curr_dof; ++m) {
      const double diff = pos_commands_[m] - pos_states_[m];
      if (std::abs(diff) > START_POS_TOLERANCE_RAD) {
        all_zero = false;
      }

      const double max_step = std::min(POS_JUMP_TOLERANCE_RAD, std::abs(diff));
      double command = pos_states_[m];
      if (pos_states_[m] < pos_commands_[m]) {
        command += max_step;
      } else {
        command -= max_step;
      }
      motor_control_->controlMIT(*motors_[m], KP[m], KD[m], command, 0.0,
                                 armGravityTorque(m));
    }
    if (all_zero) {
      zeroed = true;
    } else {
      sleep(0.01);
      read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    }
  }

  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  gripper_grasp_hold_ = false;
  gripper_stall_count_ = 0;
  gripper_hold_pos_m_ = GRIPPER_POS_CLOSED_M;
  gripper_filtered_cmd_m_ = GRIPPER_POS_CLOSED_M;
  refresh_motors();
  for (const auto& motor : motors_) {
    motor_control_->disable(*motor);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  motor_control_->recvAll();

  for (size_t i = 0; i < ARM_DOF; ++i) {
    pos_states_[i] = motors_[i]->getPosition();
    vel_states_[i] = motors_[i]->getVelocity();
    tau_states_[i] = motors_[i]->getTorque();
  }
  if (USING_GRIPPER) {
    pos_states_[GRIPPER_INDEX] = -motors_[GRIPPER_INDEX]->getPosition() *
                                 GRIPPER_REFERENCE_GEAR_RADIUS_M *
                                 GRIPPER_GEAR_DIRECTION_MULTIPLIER;
    vel_states_[GRIPPER_INDEX] = motors_[GRIPPER_INDEX]->getVelocity() *
                                 GRIPPER_REFERENCE_GEAR_RADIUS_M *
                                 GRIPPER_GEAR_DIRECTION_MULTIPLIER;
    tau_states_[GRIPPER_INDEX] = motors_[GRIPPER_INDEX]->getTorque() *
                                 GRIPPER_REFERENCE_GEAR_RADIUS_M *
                                 GRIPPER_GEAR_DIRECTION_MULTIPLIER;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  if (disable_torque_) {
    // refresh motor state on write
    for (size_t i = 0; i < curr_dof; ++i) {
      motor_control_->controlMIT(*motors_[i], 0.0, 0.0, 0.0, 0.0, 0.0);
      return hardware_interface::return_type::OK;
    }
  }

  for (size_t i = 0; i < ARM_DOF; ++i) {
    if (std::abs(pos_commands_[i] - pos_states_[i]) > POS_JUMP_TOLERANCE_RAD) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "Position jump detected for joint %s: %f -> %f",
                   info_.joints[i].name.c_str(), pos_states_[i],
                   pos_commands_[i]);
      return hardware_interface::return_type::ERROR;
    }
    const double tau_ff = tau_ff_commands_[i] + armGravityTorque(i);
    motor_control_->controlMIT(*motors_[i], KP.at(i), KD.at(i),
                               pos_commands_[i], vel_commands_[i], tau_ff);
  }
  if (USING_GRIPPER) {
    const double target_cmd = std::clamp(
        pos_commands_[GRIPPER_INDEX], GRIPPER_POS_CLOSED_M, GRIPPER_POS_OPEN_M);
    pos_commands_[GRIPPER_INDEX] = target_cmd;

    const double pos = pos_states_[GRIPPER_INDEX];
    const double vel = vel_states_[GRIPPER_INDEX];
    const double force_n = std::max(0.0, gripper_force_commands_[GRIPPER_INDEX]);
    const double force_scale = std::clamp(
        force_n / GRIPPER_FORCE_FULL_SCALE_N, 0.0, 1.0);
    const double requested_speed = gripper_speed_commands_[GRIPPER_INDEX];
    const double speed_scale = (std::isfinite(requested_speed) && requested_speed > 0.0)
        ? std::clamp(std::abs(requested_speed), 0.05, 1.0)
        : 1.0;
    const double dt = std::max(0.001, period.seconds());
    const double max_step = GRIPPER_MAX_SPEED_M_S * speed_scale * dt;

    if (!std::isfinite(gripper_filtered_cmd_m_)) {
      gripper_filtered_cmd_m_ = pos;
    }
    if (std::abs(gripper_filtered_cmd_m_ - pos) > 0.02) {
      gripper_filtered_cmd_m_ = pos;
    }
    const double delta = target_cmd - gripper_filtered_cmd_m_;
    gripper_filtered_cmd_m_ += std::clamp(delta, -max_step, max_step);
    double cmd = std::clamp(
        gripper_filtered_cmd_m_, GRIPPER_POS_CLOSED_M, GRIPPER_POS_OPEN_M);

    const double err = cmd - pos;
    const bool closing = cmd < pos - 1e-5;
    const bool opening = cmd > pos + 1e-5;

    if (opening && cmd > GRIPPER_POS_OPEN_M * 0.5) {
      gripper_grasp_hold_ = false;
      gripper_stall_count_ = 0;
      gripper_hold_pos_m_ = GRIPPER_POS_CLOSED_M;
    }

    double kp = KP.at(GRIPPER_INDEX);
    double kd = KD.at(GRIPPER_INDEX);
    double motor_q =
        -cmd / GRIPPER_REFERENCE_GEAR_RADIUS_M * GRIPPER_GEAR_DIRECTION_MULTIPLIER;

    if (gripper_grasp_hold_) {
      const double hold_pos = std::clamp(
          gripper_hold_pos_m_, GRIPPER_POS_CLOSED_M, GRIPPER_POS_OPEN_M);
      motor_q = -hold_pos / GRIPPER_REFERENCE_GEAR_RADIUS_M *
                GRIPPER_GEAR_DIRECTION_MULTIPLIER;
      kp = GRIPPER_HOLD_KP_MIN +
           (GRIPPER_HOLD_KP_MAX - GRIPPER_HOLD_KP_MIN) * force_scale;
      kd = GRIPPER_GRASP_KD;
    } else {
      if (closing && std::abs(err) > GRIPPER_STALL_ERROR_M &&
          std::abs(vel) < GRIPPER_STALL_VEL_M_S) {
        if (++gripper_stall_count_ >= GRIPPER_STALL_CYCLES) {
          gripper_grasp_hold_ = true;
          gripper_stall_count_ = 0;
          const double preload = GRIPPER_HOLD_PRELOAD_MIN_M +
              (GRIPPER_HOLD_PRELOAD_MAX_M - GRIPPER_HOLD_PRELOAD_MIN_M) *
                  force_scale;
          // Freeze the hold target slightly inside the contact point.  Updating
          // it from live feedback would follow bottle/object rebound outward and
          // look like the gripper is releasing after a successful grasp.
          gripper_hold_pos_m_ = std::clamp(
              pos - preload,
              GRIPPER_POS_CLOSED_M,
              GRIPPER_POS_OPEN_M);
          gripper_filtered_cmd_m_ = gripper_hold_pos_m_;
          motor_q = -gripper_hold_pos_m_ / GRIPPER_REFERENCE_GEAR_RADIUS_M *
                    GRIPPER_GEAR_DIRECTION_MULTIPLIER;
          kp = GRIPPER_HOLD_KP_MIN +
               (GRIPPER_HOLD_KP_MAX - GRIPPER_HOLD_KP_MIN) * force_scale;
          kd = GRIPPER_GRASP_KD;
          RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                      "Gripper grasp hold: contact=%.4fm hold=%.4fm force=%.2fN speed=%.2f kp=%.1f",
                      pos, gripper_hold_pos_m_, force_n, speed_scale, kp);
        }
      } else {
        gripper_stall_count_ = 0;
      }

      if (!gripper_grasp_hold_) {
        if (opening && pos >= GRIPPER_POS_OPEN_M - GRIPPER_STALL_ERROR_M &&
            err > GRIPPER_STALL_ERROR_M) {
          motor_q = -GRIPPER_POS_OPEN_M / GRIPPER_REFERENCE_GEAR_RADIUS_M *
                    GRIPPER_GEAR_DIRECTION_MULTIPLIER;
          kp = GRIPPER_LIMIT_KP;
        } else if (closing && pos <= GRIPPER_POS_CLOSED_M + GRIPPER_STALL_ERROR_M &&
                   err < -GRIPPER_STALL_ERROR_M) {
          motor_q = -GRIPPER_POS_CLOSED_M / GRIPPER_REFERENCE_GEAR_RADIUS_M *
                    GRIPPER_GEAR_DIRECTION_MULTIPLIER;
          kp = GRIPPER_LIMIT_KP;
        }
      }
    }

    motor_control_->controlMIT(
        *motors_[GRIPPER_INDEX], kp, kd, motor_q,
        vel_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M *
            GRIPPER_GEAR_DIRECTION_MULTIPLIER,
        tau_ff_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M *
            GRIPPER_GEAR_DIRECTION_MULTIPLIER);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmHW,
                       hardware_interface::SystemInterface)
