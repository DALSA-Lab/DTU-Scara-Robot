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

#include "bioscara_hardware_interface/bioscara_hardware.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bioscara_hardware_interface
{
  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    /**
     * Loop over all joints decribed in the hardware description file, check if they have only the position command
     * and state interface defined and finally add them to the internal Joints_ list
     *
     */
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // expect only one command interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect the command interface to be position
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect only one state interface
      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect state interface to be position
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // add joint one by one reading parameters from urdf
      Joints_.addJoint(
          joint.name,
          std::stoi(joint.parameters.at("i2c_address")),
          std::stof(joint.parameters.at("gear_ratio")),
          std::stof(joint.parameters.at("offset")));
    }

    /**
     * @todo
     * - Implement sensors and GPIO
     */
    // // GPIO components
    // if (info_.gpios.size() != 2)
    // {
    //   RCLCPP_FATAL(
    //       get_logger(), "BioscaraHardwareInterface has '%ld' GPIO components, '%d' expected.",
    //       info_.gpios.size(), 2);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
    // // with exactly 1 command interface
    // for (int i = 0; i < 2; i++)
    // {
    //   if (info_.gpios[i].command_interfaces.size() != 1)
    //   {
    //     RCLCPP_FATAL(
    //         get_logger(), "GPIO component %s has '%ld' command interfaces, '%d' expected.",
    //         info_.gpios[i].name.c_str(), info_.gpios[i].command_interfaces.size(), 1);
    //     return hardware_interface::CallbackReturn::ERROR;
    //   }
    // }
    // // and 3/1 state interfaces, respectively
    // if (info_.gpios[0].state_interfaces.size() != 3)
    // {
    //   RCLCPP_FATAL(
    //       get_logger(), "GPIO component %s has '%ld' state interfaces, '%d' expected.",
    //       info_.gpios[0].name.c_str(), info_.gpios[0].state_interfaces.size(), 3);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
    // if (info_.gpios[1].state_interfaces.size() != 1)
    // {
    //   RCLCPP_FATAL(
    //       get_logger(), "GPIO component %s has '%ld' state interfaces, '%d' expected.",
    //       info_.gpios[1].name.c_str(), info_.gpios[1].state_interfaces.size(), 1);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Called on transition to FINALIZED state
   *
   * Removes all joints from the com object.
   *
   * @todo Research in ROS2_control source code if this is ever called from any state other than
   * UNCONFIGURED
   *
   */
  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    Joints_.removeJoints();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    /**
     * Connect to the joints and throw error if it fails
     *
     */
    if (Joints_.init() < 0)
    {
      RCLCPP_FATAL(
          get_logger(),
          "Failed to connect to a joint. Check error output for more information");
      return CallbackReturn::ERROR;
    }

    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, 0.0);
    }
    // for (const auto &[name, descr] : gpio_state_interfaces_)
    // {
    //   set_state(name, 0.0);
    // }
    // for (const auto &[name, descr] : gpio_command_interfaces_)
    // {
    //   set_command(name, 0.0);
    // }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    /**
     * Disconnect from the joints and throw error if it fails
     *
     */
    if (Joints_.deinit() != 0)
    {
      RCLCPP_FATAL(
          get_logger(),
          "Failed to disconnect from a joint. Check error output for more information");
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      Joints_.enable(name,
                     std::stoi(descr.interface_info.parameters.at("drive_current")),
                     std::stoi(descr.interface_info.parameters.at("hold_current")));

      Joints_.enableStallguard(name,
                               std::stoi(descr.interface_info.parameters.at("stall_threshold")));

      Joints_.setMaxAcceleration(name,
                                 std::stof(descr.interface_info.parameters.at("max_acceleration")));
      Joints_.setMaxVelocity(name,
                             std::stof(descr.interface_info.parameters.at("max_velocity")));
    }

    // command and state should be equal when starting
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_command(name, get_state(name));
    }
    // for (const auto &[name, descr] : gpio_command_interfaces_)
    // {
    //   set_command(name, get_state(name));
    // }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    Joints_.disables();
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BioscaraHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      float v;
      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        if (Joints_.getPosition(name, v) < 0)
        {
          RCLCPP_FATAL(
              get_logger(),
              "Failed to read position from joint. Check error output for more information");
          return hardware_interface::return_type::ERROR;
        }
      }
      else if (descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        if (Joints_.getVelocity(name, v) < 0)
        {
          RCLCPP_FATAL(
              get_logger(),
              "Failed to read velocity from joint. Check error output for more information");
          return hardware_interface::return_type::ERROR;
        }
      }
      set_state(name, (double)v);
    }

    // for (const auto &[name, descr] : gpio_command_interfaces_)
    // {
    //   // mirror GPIOs back
    //   set_state(name, get_command(name));
    // }

    // // random inputs analog_input1 and analog_input2
    // unsigned int seed = static_cast<unsigned int>(time(NULL)) + 1;
    // set_state(
    //     info_.gpios[0].name + "/" + info_.gpios[0].state_interfaces[1].name,
    //     static_cast<double>(rand_r(&seed)));
    // seed = static_cast<unsigned int>(time(NULL)) + 2;
    // set_state(
    //     info_.gpios[0].name + "/" + info_.gpios[0].state_interfaces[2].name,
    //     static_cast<double>(rand_r(&seed)));

    // for (const auto &[name, descr] : gpio_state_interfaces_)
    // {
    //   ss << std::fixed << std::setprecision(2) << std::endl
    //      << "\t" << get_state(name) << " from GPIO input '" << name << "'";
    // }
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BioscaraHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      float v;
      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        if (Joints_.setPosition(name, (float)get_command(v)) < 0)
        {
          RCLCPP_FATAL(
              get_logger(),
              "Failed to set position of joint. Check error output for more information");
          return hardware_interface::return_type::ERROR;
        }
      }
      else if (descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        if (Joints_.setVelocity(name, (float)get_command(v)) < 0)
        {
          RCLCPP_FATAL(
              get_logger(),
              "Failed to set velocity of joint. Check error output for more information");
          return hardware_interface::return_type::ERROR;
        }
      }
    }
    // for (const auto &[name, descr] : gpio_command_interfaces_)
    // {
    //   // Simulate sending commands to the hardware
    //   ss << std::fixed << std::setprecision(2) << std::endl
    //      << "\t" << get_command(name) << " for GPIO output '" << name << "'";
    // }
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    return hardware_interface::return_type::OK;
  }

} // namespace bioscara_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bioscara_hardware_interface::BioscaraHardwareInterface, hardware_interface::SystemInterface)
