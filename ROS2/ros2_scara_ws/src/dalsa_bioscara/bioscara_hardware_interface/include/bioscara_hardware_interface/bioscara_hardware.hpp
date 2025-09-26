// Copyright 2023 ros2_control Development Team
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

#ifndef BIOSCARA_HARDWARE_INTERFACE_HPP_
#define BIOSCARA_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "bioscara_hardware_driver/mJointCom.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace bioscara_hardware_interface
{
  class BioscaraHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BioscaraHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    /*
     * Communication object containing all joints of the robot
     */
    Joint_comms Joints_;

    double deg2rad(float )
  };

} // namespace bioscara_hardware_interface

#endif // BIOSCARA_HARDWARE_INTERFACE_HPP_
