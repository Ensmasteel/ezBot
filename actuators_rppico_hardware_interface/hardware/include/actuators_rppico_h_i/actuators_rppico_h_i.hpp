// Copyright 2021 ros2_control Development Team
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

#ifndef ACTUATORS_RPPICO_H_I__OMNIBOT_SYSTEM_HPP_
#define ACTUATORS_RPPICO_H_I__OMNIBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "actuators_rppico_h_i/visibility_control.h"

#include "actuators_rppico_h_i/rppico_comms.hpp"
#include "actuators_rppico_h_i/servo.hpp"

namespace actuators_rppico_h_i
{
class ActuatorsRpPicoHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::array<std::string, 12> servo_names = {};

  float loop_rate = 0.0;

  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;

};;


public:
  
  int number_of_servos = 0;
  
  RCLCPP_SHARED_PTR_DEFINITIONS(ActuatorsRpPicoHardware);

  ACTUATORS_RPPICO_H_I_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ACTUATORS_RPPICO_H_I_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ACTUATORS_RPPICO_H_I_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ACTUATORS_RPPICO_H_I_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ACTUATORS_RPPICO_H_I_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  ACTUATORS_RPPICO_H_I_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACTUATORS_RPPICO_H_I_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACTUATORS_RPPICO_H_I_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ACTUATORS_RPPICO_H_I_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  RpPicoComs comms_;
  Config cfg_;

  std::array<Servo, 12> servos_;

};

}  // namespace ACTUATORS_RPPICO_H_I

#endif  // ACTUATORS_RPPICO_H_I__DIFFBOT_SYSTEM_HPP_
