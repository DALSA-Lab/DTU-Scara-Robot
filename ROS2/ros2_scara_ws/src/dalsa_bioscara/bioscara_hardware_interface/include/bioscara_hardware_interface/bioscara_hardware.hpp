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
             * @brief 
             * @todo this
             * @param start_interfaces 
             * @param stop_interfaces 
             * @return hardware_interface::return_type 
             */
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    private:
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
        };

        // /**
        //  * @brief Saves a defined list of possible command modes
        //  *
        //  */
        // enum command_mode_t : std::uint8_t
        // {
        //     UNDEFINED = 0,
        //     POSITION = 1,
        //     VELOCITY = 2,
        //     ACCELERATION = 3,
        //     HOMING = 4,
        //     DIFFPOSITION = 5
        // };

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
         * by concatenating joint name with interface name.
         *
         */
        std::unordered_map<std::string,std::set<std::string>> _joint_command_modes;
    };

} // namespace bioscara_hardware_interface

#endif // BIOSCARA_HARDWARE_INTERFACE_HPP_
