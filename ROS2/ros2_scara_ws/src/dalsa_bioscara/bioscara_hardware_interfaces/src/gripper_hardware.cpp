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

#include "bioscara_hardware_interfaces/gripper_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bioscara_hardware_interfaces
{
  hardware_interface::CallbackReturn BioscaraGripperHardwareInterface::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (
        hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string use_mock_hardware = info_.hardware_parameters["use_mock_hardware"];

    try
    {
      _gripper_cfg.reduction = std::stof(info_.hardware_parameters["reduction"]);
      _gripper_cfg.offset = std::stof(info_.hardware_parameters["offset"]);
      _gripper_cfg.min = std::stof(info_.hardware_parameters["min"]);
      _gripper_cfg.max = std::stof(info_.hardware_parameters["max"]);
    }
    catch (...)
    {
      RCLCPP_FATAL(
          get_logger(), "Gripper '%s' is missing one of the following parameters: reduction, offset, min, max",
          info_.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    /**
     * check that only one joint is defined, more mimic joints may be defined.
     */
    if (info_.joints.size() != 1)
    {
      RCLCPP_FATAL(
          get_logger(), "Gripper '%s' has %zu joints. 1 expected.",
          info_.name.c_str(), info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    hardware_interface::ComponentInfo joint = info_.joints[0];

    // expect exactly one command interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
          get_logger(), "Gripper '%s' has %zu command interfaces. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // expect the first command interface to be position
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
          get_logger(), "Gripper '%s' has '%s' command interface. '%s' expected.",
          joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // expect no state interfaces, warn if there is. Wont be used.
    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_WARN(
          get_logger(), "Gripper '%s' has %zu state interfaces. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
    }

    // expect the first state interface to be position
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
          get_logger(), "Gripper '%s' has '%s' state interface. '%s' expected.",
          joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (use_mock_hardware == "true" || use_mock_hardware == "True")
    {
      _gripper = std::make_unique<bioscara_hardware_driver::MockGripper>();
    }
    else
    {
      _gripper = std::make_unique<bioscara_hardware_driver::Gripper>(_gripper_cfg.reduction,
                                                                     _gripper_cfg.offset,
                                                                     _gripper_cfg.min,
                                                                     _gripper_cfg.max);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraGripperHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_logger(), "Shutting down ...please wait...");

    hardware_interface::CallbackReturn cr = CallbackReturn::SUCCESS;
    switch (previous_state.id())
    {
      /* Joints already deinitialized */
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;

      /* Deinitialize joints first */
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      cr = on_cleanup(previous_state);
      break;
    }
    if (cr != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Shut down");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraGripperHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    bioscara_hardware_driver::err_type_t rc = _gripper->init();
    if (rc != bioscara_hardware_driver::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_driver::error_to_string(rc);
      RCLCPP_FATAL(
          get_logger(),
          "Failed to connect to gripper. Reason: %s", reason.c_str());
      return CallbackReturn::ERROR;
    }

    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_state(name, std::numeric_limits<double>::quiet_NaN());
    }
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, std::numeric_limits<double>::quiet_NaN());
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraGripperHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");

    bioscara_hardware_driver::err_type_t rc = _gripper->deinit();
    if (rc != bioscara_hardware_driver::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_driver::error_to_string(rc);
      RCLCPP_FATAL(
          get_logger(),
          "Failed to disconnect from gripper. Reason: %s", reason.c_str());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraGripperHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    // enable PWM
    bioscara_hardware_driver::err_type_t rc = _gripper->enable();
    if (rc != bioscara_hardware_driver::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_driver::error_to_string(rc);
      RCLCPP_FATAL(
          get_logger(),
          "Failed to enable gripper. Reason: %s", reason.c_str());
      return CallbackReturn::ERROR;
    }

    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      /* Check if position or velocity. Set position to current position and velocity to 0.0 */
      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_INFO(get_logger(), "Set %s, to NaN", name.c_str());
        set_command(name, std::numeric_limits<double>::quiet_NaN());
      }
      else if (descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_INFO(get_logger(), "Set %s, to 0.0", name.c_str());
        set_command(name, 0.0);
      }
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraGripperHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    bioscara_hardware_driver::err_type_t rc = _gripper->disable();
    if (rc != bioscara_hardware_driver::err_type_t::OK)
    {
      std::string reason = bioscara_hardware_driver::error_to_string(rc);
      RCLCPP_FATAL(
          get_logger(),
          "Failed to disable gripper. Reason: %s", reason.c_str());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BioscaraGripperHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    /* Simply mirror commands to states */
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      try
      {
        set_state(name, get_command(name));
      }
      catch (const std::exception &e)
      {
        RCLCPP_FATAL(
            get_logger(),
            "Failed to read state '%s'. Reason: %s", name.c_str(), e.what());
        return hardware_interface::return_type::ERROR;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BioscaraGripperHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      bioscara_hardware_driver::err_type_t rc = static_cast<bioscara_hardware_driver::err_type_t>(1);
      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        rc = _gripper->setPosition((float)get_command(name));
      }

      // use != 0 here since 1 for no compatible interface type
      if (rc != bioscara_hardware_driver::err_type_t::OK)
      {
        std::string reason = bioscara_hardware_driver::error_to_string(rc);
        RCLCPP_FATAL(
            get_logger(),
            "Failed to set %s of gripper. Reason: %s", name.c_str(), reason.c_str());
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn BioscaraGripperHardwareInterface::on_error(
      const rclcpp_lifecycle::State &previous_state)
  {
    /*
     * Call the deactivation method. If the robot successfully deactivates the hardware remains in the unconfigured state,
     * and is able to be activated again. Otherwise the hardware goes to the finalized state and can not be recovered.
     */
    RCLCPP_INFO(get_logger(), "Previous State: %s", previous_state.label().c_str());
    // states: "active", "finalized",...

    /*
     * call the deactivate function anyway regardless if state was active or inactive. For example if the on_activate function fails
     * the joint might still be enabled, to disable them invoke on_deactivate().
     */
    if (previous_state.label() == "active" || previous_state.label() == "inactive")
    {
      hardware_interface::CallbackReturn cr = on_deactivate(previous_state);
      if (cr != CallbackReturn::SUCCESS)
      {
        return cr;
      }

      /* since the hardware goes to "unconfigured state if an error is caught and the on_error function returns SUCCESS
      we also have to manually call the on_cleanup function */
      cr = on_cleanup(previous_state);
      if (cr != CallbackReturn::SUCCESS)
      {
        return cr;
      }
      return CallbackReturn::SUCCESS;
    }

    return CallbackReturn::ERROR;
  }
} // namespace bioscara_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bioscara_hardware_interfaces::BioscaraGripperHardwareInterface, hardware_interface::SystemInterface)
