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
     * Loop over all joints decribed in the hardware description file, check if they have the position and velocity command
     * and state interface defined and finally add them to the internal _joints list
     *
     */
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // expect exactly two command interface
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

      // expect exactly two state interfaces
      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect state interface to be position or velocity
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
          joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s state interfaces found. '%s' and '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // add joint one by one reading parameters from urdf

      joint_config_t cfg;
      try
      {
        cfg.i2c_address = std::stoi(joint.parameters.at("i2c_address"), nullptr, 16);
        cfg.reduction = std::stof(joint.parameters.at("reduction"));
        cfg.min = std::stof(joint.parameters.at("min"));
        cfg.max = std::stof(joint.parameters.at("max"));
        cfg.stall_threshold = std::stoi(joint.parameters.at("stall_threshold"));
        cfg.hold_current = std::stoi(joint.parameters.at("hold_current"));
        cfg.drive_current = std::stoi(joint.parameters.at("drive_current"));
        cfg.max_acceleration = std::stof(joint.parameters.at("max_acceleration"));
        cfg.max_velocity = std::stof(joint.parameters.at("max_velocity"));
      }
      catch (...)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' is missing one of the following parameters: i2c_address, reduction, min, max, stall_threshold, hold_current, drive_current, max_acceleration, max_velocity",
            joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

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
     * Loop over all GPIOs decribed in the hardware description file, check if they have the home command
     * and state interface defined.
     *
     */
    if (info_.gpios.size() != _joints.size())
    {
      RCLCPP_FATAL(
          get_logger(), "BioscaraHardwareInterface has '%ld' GPIO components, '%lu' expected. Every joint needs a GPIO interface",
          info_.gpios.size(), _joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
    {
      // expect only one command interface
      if (gpio.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "GPIO '%s' has %zu command interfaces found. 1 expected.",
            gpio.name.c_str(), gpio.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect the first command interface to be 'home'
      if (gpio.command_interfaces[0].name != bioscara_hardware_interface::HW_IF_HOME)
      {
        RCLCPP_FATAL(
            get_logger(), "GPIO '%s' have %s command interfaces found. '%s' expected.",
            gpio.name.c_str(),
            gpio.command_interfaces[0].name.c_str(),
            bioscara_hardware_interface::HW_IF_HOME);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect only one state interface
      if (gpio.state_interfaces.size() > 1)
      {
        RCLCPP_FATAL(
            get_logger(), "GPIO '%s' has %zu state interface. 1 expected.", gpio.name.c_str(),
            gpio.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // expect state interface to be 'home'
      if (gpio.state_interfaces[0].name != bioscara_hardware_interface::HW_IF_HOME)
      {
        RCLCPP_FATAL(
            get_logger(), "GPIO '%s' have %s state interfaces found. '%s' expected.",
            gpio.name.c_str(), gpio.state_interfaces[0].name.c_str(),
            bioscara_hardware_interface::HW_IF_HOME);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Check that a joint with the same name exists
      if (_joints.find(gpio.name) == _joints.end())
      {
        RCLCPP_FATAL(
            get_logger(), "No matching joint with the name '%s' found. Currently every GPIO needs to match a joint.",
            gpio.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      /**
       * @todo threshold and current are uint8_t, if a number larger outside 0 < n < 255 is passed as a parameters it will overflow.
       */
      joint_homing_config_t cfg;
      try
      {
        cfg.speed = std::stof(gpio.command_interfaces[0].parameters.at("speed"));
        cfg.threshold = std::stoi(gpio.command_interfaces[0].parameters.at("threshold"));
        cfg.current = std::stoi(gpio.command_interfaces[0].parameters.at("current"));
        cfg.acceleration = std::stof(gpio.command_interfaces[0].parameters.at("acceleration"));
      }
      catch (...)
      {
        RCLCPP_FATAL(
            get_logger(), "GPIO '%s' is missing one of the following parameters in 'home' command_interface: speed, threshold, current, acceleration",
            gpio.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      _joint_cfg[gpio.name].homing = cfg;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_shutdown(
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
    for (const auto &[name, descr] : gpio_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : gpio_command_interfaces_)
    {
      set_command(name, 0.0);
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");

    /**
     * Disconnect from the joints and throw error if it fails
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
      /* Clear the active command modes. Controllers can only be activated after the hardware is activated. */
      _joint_command_modes[name] = {};

      joint_config_t cfg = _joint_cfg[name];

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
        case -2:
          reason = "joint not homed";
          break;
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

    for (const auto &[name, descr] : gpio_state_interfaces_)
    {
      float v;
      int rc = 1;

      if (descr.interface_info.name == bioscara_hardware_interface::HW_IF_HOME)
      {
        /* Reset the return code. All following functions do not return an error code */
        rc = 0;

        /* We can assume that the previous calls to read the joint state interfaces
        gave us the latest flags. Hence we can simply retrieve the HOMED flag by calling isHomed().
        This does not generate additional communication trafic.  */
        v = _joints.at(descr.prefix_name).isHomed() * 1.0;

        /* If the homing has been activated (through the command interface) the device signals BUSY
        as long as it is still homing. If the BUSY flag is reset while the current command is still HOME
        we can assume the homing has finished. Then reset the command interface to 0, which will stop the homing (
        perform cleanup tasks) at the next write cycle. */
        if (_joints.at(descr.prefix_name).getCurrentBCmd() == Joint::HOME &&
            !_joints.at(descr.prefix_name).isBusy())
        {
          set_command(name, 0.0);
        }
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
          reason = "joint not homed";
          break;
        default:
          reason = "Unkown Reason " + std::to_string(rc);
        }
        RCLCPP_FATAL(
            get_logger(),
            "Failed to read %s of GPIO '%s'. Reason: %s", descr.interface_info.name.c_str(), name.c_str(), reason.c_str());
        return hardware_interface::return_type::ERROR;
      }
      set_state(name, (double)v);
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BioscaraHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    /* Loop over all active command interfaces for each joint and write the command to the hardware.
    Previously the command was written to every command interface specified in the ros2_control urdf. This causes conflicts. */
    for (const auto &[name, interfaces] : _joint_command_modes)
    {
      for (std::string interface : interfaces)
      {
        std::string CIF_name = name + "/" + interface;
        int rc = 1;
        if (interface == hardware_interface::HW_IF_POSITION)
        {
          rc = _joints.at(name).setPosition((float)get_command(CIF_name));
        }
        else if (interface == hardware_interface::HW_IF_VELOCITY)
        {
          rc = _joints.at(name).setVelocity((float)get_command(CIF_name));
        }
        else if (interface == bioscara_hardware_interface::HW_IF_HOME)
        {
          joint_config_t cfg = _joint_cfg[name];
          float velocity = get_command(CIF_name);
          Joint::stp_reg_t current_cmd = _joints.at(name).getCurrentBCmd();

          /* If the joint is currently executing a homing call and the velocity is set to 0.0,
          stop the homing. This could be either because the user interrupts the homing or the homing is finished.*/
          if (velocity == 0.0)
          {
            if (current_cmd == Joint::HOME)
            {
              /* Stop the homing. Reset acceleration and perform the postHoming cleanup */
              _joints.at(name).setMaxAcceleration(cfg.max_acceleration);
              _joints.at(name).postHoming();
              rc = _joints.at(name).stop();
            }
          }

          /* If the command is != 0.0 and the joint is currently not executing a blocking command,
          most likely the homing itself from a previous call, start the homing sequence. */
          else
          {
            if (current_cmd == Joint::NONE)
            {
              float speed = velocity > 0.0 ? cfg.homing.speed : -cfg.homing.speed;
              _joints.at(name).setMaxAcceleration(cfg.homing.acceleration);
              rc = _joints.at(name).startHoming(speed, cfg.homing.threshold, cfg.homing.current);
            }
            else if (current_cmd != Joint::HOME)
            {
              rc = -109;
            }
          }
        }
        // use != 0 here since 1 for no compatible interface type
        if (rc != 0)
        {
          std::string reason = "";
          switch (rc)
          {
          case 1:
            reason = "no compatible command to read " + interface;
            break;
          case -1:
            reason = "communication error";
            break;
          case -2:
            reason = "joint not homed, can not set " + interface;
            break;
          case -3:
            reason = "joint not enabled, can not set " + interface;
            break;
          case -4:
            reason = "joint stalled, can not set " + interface;
            break;
          case -109:
            reason = "joint busy processing another blocking command";
            break;
          default:
            reason = "Unkown Reason " + std::to_string(rc);
          }
          RCLCPP_FATAL(
              get_logger(),
              "Failed to set %s of joint '%s'. Reason: %s", CIF_name.c_str(), name.c_str(), reason.c_str());
          return hardware_interface::return_type::ERROR;
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BioscaraHardwareInterface::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {

    std::unordered_map<std::string, std::set<std::string>> new_active_interfaces = _joint_command_modes;

    /* First remove all stopped interfaces from the active set */
    for (std::string interface : stop_interfaces)
    {
      std::string full_interface = interface;
      /* split 'joint/interface to 'joint' and 'interface' */
      std::string delimiter = "/";
      size_t pos = interface.find(delimiter);
      std::string joint = interface.substr(0, pos);
      interface.erase(0, pos + delimiter.length());

      /* If the interface that is to be stopped is the homing interface, check that no current homing command is active */
      if (interface == bioscara_hardware_interface::HW_IF_HOME &&
          _joints.at(joint).getCurrentBCmd() == Joint::HOME)
      {
        RCLCPP_FATAL(
            get_logger(),
            "The controller tried to deactivate '%s' of '%s' but homing is still ongoing.", interface.c_str(), joint.c_str());
        return hardware_interface::return_type::ERROR;
      }

      /* If the interface that is to be stopped is the velocity interface, check that the velocity is 0.0 */
      if (interface == hardware_interface::HW_IF_VELOCITY &&
          get_command(full_interface) != 0.0)
      {
        RCLCPP_WARN(
            get_logger(),
            "The controller tried to deactivate '%s' of '%s' but the velocity is not 0.0", interface.c_str(), joint.c_str());
      }

      if (new_active_interfaces.at(joint).erase(interface) == 0)
      {
        RCLCPP_WARN(
            get_logger(),
            "The controller tried to stop the interface '%s' of '%s' but it has not been started.", interface.c_str(), joint.c_str());
      }
    }

    /* Then add all new interfaces to the active set */
    for (std::string interface : start_interfaces)
    {
      /* split 'joint/interface to 'joint' and 'interface' */
      std::string delimiter = "/";
      size_t pos = interface.find(delimiter);
      std::string joint = interface.substr(0, pos);
      interface.erase(0, pos + delimiter.length());

      std::pair rc = new_active_interfaces.at(joint).insert(interface);
      if (rc.second == 0)
      {
        RCLCPP_FATAL(
            get_logger(),
            "The controller tried to start the interface '%s' of '%s' but it has already been started by another controller.", interface.c_str(), joint.c_str());
        return hardware_interface::return_type::ERROR;
      }
    }

    /* Validation. Currently the validation is very simple, for every joint only one active interface must exist.*/
    for (auto &[name, interfaces] : new_active_interfaces)
    {
      size_t size = interfaces.size();
      if (size > 1)
      {
        std::string active_if = "";
        for (auto interface : interfaces)
        {
          active_if += (interface + "\n");
        }
        RCLCPP_FATAL(
            get_logger(),
            "The controller tries to start multiple command interfaces for '%s'. The following interfaces are active or are trying to be active:\n%s.",
            name.c_str(), active_if.c_str());
        return hardware_interface::return_type::ERROR;
      }
    }

    /* If the command mode switch was successfull save the new active interfaces */
    _joint_command_modes = new_active_interfaces;

    std::string active_if = "";
    for (const auto &[name, interfaces] : new_active_interfaces)
    {
      active_if += (name + ":\n[\n");
      for (std::string interface : interfaces)
      {
        active_if += ("\t" + interface + "\n");
      }
      active_if += ("]\n");
    }

    RCLCPP_INFO(
        get_logger(),
        "New active command modes:\n%s", active_if.c_str());

    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn BioscaraHardwareInterface::on_error(
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

} // namespace bioscara_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    bioscara_hardware_interface::BioscaraHardwareInterface, hardware_interface::SystemInterface)
