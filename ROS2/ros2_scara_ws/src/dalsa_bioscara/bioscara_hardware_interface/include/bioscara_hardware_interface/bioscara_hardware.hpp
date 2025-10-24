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
#include <set>
#include <unordered_map>

#include "bioscara_hardware_driver/mJoint.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace bioscara_hardware_interface
{
    constexpr char HW_IF_HOME[] = "home";

    class BioscaraHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(BioscaraHardwareInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

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
            const rclcpp::Time &time, 
            const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, 
            const rclcpp::Duration &period) override;

        /**
         * @brief Performs checks and book keeping of the active control mode when changing controllers.
         * 
         * For safe operation only one controller may interact with the hardware at the time.
         * For example if the velocity JTC is active and has claimed the velocity command interfaces it is technically possible to 
         * activate the position JTC (or a homing controller, or others) that claim a different command interface (position in this case).
         * However if both controllers are active they start writing to the hardware simultaneously which is to be avoided.
         * For this reason a book keeping mechanism has been implemented which stores the currently active command interfaces for each joint in the
         * _joint_command_modes member. Each joint has a set of active command interfaces. When a controller switch is performed first the interfaces that should be stopped are removed from
         * each joint set, then the one that should be started are added, if they are already present an error is thrown. Lastly
         * a validation is performed. Currently the validation is simple since each joint may only have one command interface. The validation can be expanded for furture use cases that require
         * a combination of active command interfaces per joint for example.
         * 
         * @param start_interfaces command interfaces that should be started in the form "joint/interface"
         * @param stop_interfaces command interfaces that should be stopped in the form "joint/interface" 
         * @return hardware_interface::return_type 
         */
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    private:

        /**
         * @brief configuration structure holding the passed homing paramters from the ros2_control urdf
         *
         * Saving all parameters on initialization in a structure allows for quick access during runtime.
         *
         */
        struct joint_homing_config_t
        {
            float speed = 0;
            u_int8_t threshold = 10;
            u_int8_t current = 10;
            float acceleration = 0.01;
        };

        /**
         * @brief configuration structure holding the passed paramters from the ros2_control urdf
         *
         * Saving all parameters on initialization in a structure allows for quick access during runtime.
         *
         */
        struct joint_config_t
        {
            int i2c_address;
            float reduction = 1;
            float min;
            float max;
            u_int8_t drive_current;
            u_int8_t hold_current;
            u_int8_t stall_threshold;
            float max_velocity;
            float max_acceleration;
            joint_homing_config_t homing;
        };

        /**
         * @brief unordered map storing the Joint objects.
         *
         * an unordered map is chosen to simplify acces via the joint name, as this conforms well with the ROS2_control hardware interface
         * The map does not need to be ordered. Search, insertion, and removal of elements have average constant-time complexity.
         *
         */
        std::unordered_map<std::string, Joint> _joints;

        /**
         * @brief unordered map storing the configuration struct of the joints.
         *
         * an unordered map is chosen to simplify acces via the joint name, as this conforms well with the ROS2_control hardware interface
         * The map does not need to be ordered. Search, insertion, and removal of elements have average constant-time complexity.
         *
         */
        std::unordered_map<std::string, joint_config_t> _joint_cfg;

        /**
         * @brief unordered map of sets storing the active command interfaces for each joint.
         *
         * an unordered map is chosen to simplify acces via the joint name, as this conforms well with the ROS2_control hardware interface
         * The map does not need to be ordered. Search, insertion, and removal of elements have average constant-time complexity. Each joint can have a set
         * of active joint interfaces. This type of structure is chosen to group interfaces by joint. In the write() function the interface name can simply be constructed
         * by concatenating joint name with interface name. Although currently only one active command interface is allowed at the time, a set can be used to store multiple
         * command interfaces that are acceptable to be combined, for example if it would be acceptable to set velocity and driver current and hence that would be an allowable
         * combination.
         *
         */
        std::unordered_map<std::string,std::set<std::string>> _joint_command_modes;
    };

} // namespace bioscara_hardware_interface

#endif // BIOSCARA_HARDWARE_INTERFACE_HPP_
