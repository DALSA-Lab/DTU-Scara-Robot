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
      const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (
        hardware_interface::SystemInterface::on_init(params) !=
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
      if (joint.command_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 2 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect the first command interface to be position
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), 
            joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect the second command interface to be velocity
      if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), 
            joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect only one state interface
      if (joint.state_interfaces.size() > 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 2 or less expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect state interface to be position or velocity
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
          joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s state interfaces found. '%s' or '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // add joint one by one reading parameters from urdf

      joint_config_t cfg;
      cfg.i2c_address = std::stoi(joint.parameters.at("i2c_address"), nullptr, 16),
      cfg.reduction = std::stof(joint.parameters.at("reduction")),
      cfg.min = std::stof(joint.parameters.at("min")),
      cfg.max = std::stof(joint.parameters.at("max")),
      cfg.stall_threshold = std::stoi(joint.parameters.at("stall_threshold")),
      cfg.hold_current = std::stoi(joint.parameters.at("hold_current")),
      cfg.drive_current = std::stoi(joint.parameters.at("drive_current")),
      cfg.max_acceleration = std::stof(joint.parameters.at("max_acceleration")),
      cfg.max_velocity = std::stof(joint.parameters.at("max_velocity")),

      _joint_cfg.insert({joint.name, cfg});

      _joints.insert({joint.name,
                      Joint(
                          joint.name,
                          cfg.i2c_address,
                          cfg.reduction,
                          cfg.min,
                        cfg.max)});
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
    RCLCPP_INFO(get_logger(), "Shutting down ...please wait...");
    _joints.clear();
    RCLCPP_INFO(get_logger(), "Shut down");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    // Init each joint and test connection to each joint by pinging
    for (auto &[name, joint] : _joints)
    {
      int rc = joint.init();
      if (rc < 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case -1:
          reason = "Joint added to I2C bus but no ACK received back from the joint when pinged";
          break;
        case -2:
          reason = "I2C error";
          break;

        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to connect to joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }
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
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");

    /**
     * Disconnect from the joints and throw error if it fails
     *
     */
    for (auto &[name, joint] : _joints)
    {
      int rc = joint.deinit();
      if (rc < 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case -1:
          reason = "I2C error";
          break;
        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to disconnect from joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    for (auto &[name, joint] : _joints)
    {
      joint_config_t cfg = _joint_cfg[name];

      /* First get the flags. they must all be zero to indicate that the joint is operational.
      Below code does not look great*/
      int flags = joint.getFlags();
      if (flags < 0 || !joint.isHomed())
      {
        std::string reason = "";
        if (flags < 0)
        {
          reason = "communication error";
        }
        else if (!joint.isHomed())
        {
          reason = "not homed";
        }
        else
        {
          reason = "Unkown Reason (" + std::to_string(flags) + ")";
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to enable joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }

      // enable motor
      int rc = joint.enable(cfg.drive_current, cfg.hold_current);
      if (rc < 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case -1:
          reason = "communication error";
          break;
        case -3:
          reason = "motor failed to enable";
          break;
        default:
          reason = "Unkown Reason" + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to enable joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }

      // enable stall detection
      rc = joint.enableStallguard(cfg.stall_threshold);
      if (rc < 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case -1:
          reason = "communication error";
          break;

        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to enable stall protection of joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }

      // set max acceleration
      rc = joint.setMaxAcceleration(cfg.max_acceleration);
      if (rc < 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case -1:
          reason = "communication error";
          break;

        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to set maximum acceleration of joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }

      // set max velocity
      rc = joint.setMaxVelocity(cfg.max_velocity);
      if (rc < 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case -1:
          reason = "communication error";
          break;

        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to set maximum velocity of joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }
    }

    /**
     * Below a workaround to force a read cycle of all joints to get inital values for the state interfaces.
     * These will be copied to the command interface to prevent movement at startup.
     */
    rclcpp::Time t(0);
    rclcpp::Duration d(0, 0);
    if (read(t, d) != hardware_interface::return_type::OK)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // command and state should be equal when starting
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      /* Check if position or velocity. Set position to current position and velocity to 0.0 */
      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_INFO(get_logger(), "Set %s, to %f", name.c_str(), get_state(name));
        set_command(name, get_state(name));
      }
      else if (descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_INFO(get_logger(), "Set %s, to 0.0", name.c_str());
        set_command(name, 0.0);
      }
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
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    /**
     * disable the joints and throw error if it fails
     *
     */
    for (auto &[name, joint] : _joints)
    {
      int rc = joint.disable();
      if (rc < 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case -1:
          reason = "communication error";
          break;
        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to disable joint '%s'. Reason: %s", name.c_str(), reason.c_str());
        return CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BioscaraHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      float v;
      int rc = 1;

      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        rc = _joints.at(descr.prefix_name).getPosition(v);
      }
      else if (descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        rc = _joints.at(descr.prefix_name).getVelocity(v);
      }
      // use != 0 here since 1 for no compatible interface type
      if (rc != 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case 1:
          reason = "no compatible command to read " + descr.interface_info.name;
          break;
        case -1:
          reason = "communication error";
          break;
        // case -2:
        //   reason = "joint not homed";
        //   break;
        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to read %s of joint '%s'. Reason: %s", descr.interface_info.name.c_str(), name.c_str(), reason.c_str());
        return hardware_interface::return_type::ERROR;
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
      int rc = 1;
      if (descr.interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        rc = _joints.at(descr.prefix_name).setPosition((float)get_command(name));
      }
      else if (descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        rc = _joints.at(descr.prefix_name).setVelocity((float)get_command(name));
      }
      // use != 0 here since 1 for no compatible interface type
      if (rc != 0)
      {
        std::string reason = "";
        switch (rc)
        {
        case 1:
          reason = "no compatible command to read " + descr.interface_info.name;
          break;
        case -1:
          reason = "communication error";
          break;
        case -2:
          reason = "joint not homed, can not set " + descr.interface_info.name;
          break;
        case -3:
          reason = "joint not enabled, can not set " + descr.interface_info.name;
          break;
        case -4:
          reason = "joint stalled, can not set " + descr.interface_info.name;
          break;
        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to set %s of joint '%s'. Reason: %s", descr.interface_info.name.c_str(), name.c_str(), reason.c_str());
        return hardware_interface::return_type::ERROR;
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

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_error(
      const rclcpp_lifecycle::State &previous_state)
  {
    /**
     * Call the deactivation method. If the robot successfully deactivates the hardware remains in the unconfigured state,
     * and is able to be activated again. Otherwise the hardware goes to the finalized state and can not be recovered.
     *
     * @todo implement a more fine tuned error handling.
     */

    RCLCPP_INFO(get_logger(), "Previous State: %s", previous_state.label().c_str());
    // states: "active", "finalized",...

    /**
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

} // namespace bioscara_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bioscara_hardware_interface::BioscaraHardwareInterface, hardware_interface::SystemInterface)
