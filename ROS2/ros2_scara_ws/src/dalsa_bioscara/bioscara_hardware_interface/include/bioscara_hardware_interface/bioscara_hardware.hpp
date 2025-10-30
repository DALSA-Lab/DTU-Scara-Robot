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

    /**
     * @brief The bioscara hardware interface class.
     * 
     * The hardware interface serves to wrap custom hardware interaction in the standardized ros2_control architecture.
     *
     *
     * <b>Hardware Lifecycle</b> \n
     * The hardware follows the ros2_control hardware interface lifecyle which intern is following the [ROS2 managed node lifecycle](https://github.com/ros2/design/blob/93a415bf928751d37fce6f83215c521658e20c93/articles/node_lifecycle.md).
     *
     *
     * \image html images/hardware_interface_lifecycle.png "Hardware interface lifecycle" width=1000em
     * \image latex images/hardware_interface_lifecycle.png "Hardware interface lifecycle" width=\linewidth
     */
    class BioscaraHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(BioscaraHardwareInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        /**
         * @brief Called on the transistion from the `inactive`, `unconfigured` and `active` to the `finalized` state.
         *
         * When transitioning directly from `active` to `finalized` on_deactivate() is automatically called before [Source Code](https://github.com/ros-controls/ros2_control/blob/d0836b7f12b89acb89bde83d4ba4308513b03204/hardware_interface/src/resource_manager.cpp#L616)
         * If the previous state is either `inactive` or `active` the on_cleanup() method is called first. Then regardless of the previous state,
         * the _joints map is cleared.
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `unconfigured` to the `inactive` state.
         *
         * Establish and test connection to each joint.
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `inactive` to the `unconfigured` state.
         *
         * Disconnect from the joints.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `inactive` to the `active` state.
         *
         * Enables each joint, enables the stall detection and sets the maximmum acceleration. \n
         * It is allowed to activate the hardware even if it is not homed. To home the joint the homing_controller must be activated,
         * but generally a hardware component must be active in order for controllers to become active. \n
         *
         * To prohibit movement on activation the set point for each position command interface is set equal to the current measured position,
         * and the velocity command is set to 0.0 for each command interface. The current values are obtained by calling the read() method once
         * which populates the state interfaces with values.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Called on the transistion from the `active` to the `inactive` state.
         * 
         * Disables all joints and thereby allows backdriving. State interfaces continue to be updated.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

            /**
             * @brief Reads from the hardware and populates the state interfaces.
             * 
             * Iterates over all state interfaces and calls the corresponding Joint method.
             * 
             * - State interface "position" -> Joint::getPosition()
             * - State interface "velocity" -> Joint::getVelocity()
             * - State interface "home"     -> Joint::isHomed()
             *  - This does not actually trigger a communication, instead it relies on the return flags of 
             *    the previous transmissions. Since position and velocity have been called immediatly before the return flags
             *    are assumed to be valid.
             *  - If the the homing of a joint has been activated through the command interface (Joint::getCurrentBCmd() == Joint::HOME)
             *    the device signals BUSY (Joint::isBusy()) as long as it is still homing. \n
             *    If the BUSY flag is reset while the current command is still Joint::HOME we can assume the homing has finished. 
             *    Then the "home" command interface of the joint is reset to 0.0, which will stop the homing (perform cleanup tasks) at the next write cycle.
             *  
             * @param time 
             * @param period 
             * @return hardware_interface::return_type 
             */
        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

            /**
             * @brief Writes commands to the hardware from the command interfaces.
             * 
             * In contrast to the read() method the write() method only loops over the command interfaces that are currently active defined by
             * the BioscaraHardwareInterface::_joint_command_modes map. See prepare_command_mode_switch() for a detailed reasoning why this approach
             * has been chosen. 
             * 
             * - Command interface "position" -> Joint::setPosition()
             * - Command interface "velocity" -> Joint::setVelocity()
             * - Command interface "home"     -> Joint::startHoming()
             *  - If the commanded value in "home" is != 0.0 the and the joint is currently executing a blocking function, 
             * for example homing (Joint::getCurrentBCmd() == Joint::NONE), the homing sequence is started with the speed, sensitivity, current and acceleration
             * defined in the BioscaraHardwareInterface::_joint_cfg which is polulated from the hardware description urdf. The direction of
             * the homing is determined by the sign of the command interface value.
             *  - If the commanded value in "home" is = 0.0 and the joint is currently executing homing, the homing is stopped. This can either
             * happen prematurely through user input or when the homing is completed which is registered in read(). 
             * .
             * @param time 
             * @param period 
             * @return hardware_interface::return_type 
             */
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
         * _joint_command_modes member. Each joint has a set of active command interfaces. When a controller switch is performed the interfaces that should be stopped are removed from
         * each joint set, then the one that should be started are added, if they are already present an error is thrown. Lastly
         * a validation is performed. Currently the validation is simple since each joint may only have one command interface. The validation can be expanded for furture use cases that require
         * a combination of active command interfaces per joint for example. \n
         *
         * The following basic checks are implemented:
         * - <b>On deactivation</b>:
         *  - [ERROR] Homing command interfaces may only be deactivated if no current homing process is ongoing (Joint::getCurrentBCmd() != Joint::HOME)
         *  - [WARN] Deactivating a velocity command interface if the velocity set point is 0.0.
         *  - [WARN] Deactivating a command interface that has not been started. This should not happen.
         *
         * - <b>On activation</b>:
         *  - [ERROR] Activating a command interface that is already started. This should not happen.
         *  - [ERROR] Activating a second command interface for a joint.
         * .
         * @param start_interfaces command interfaces that should be started in the form "joint/interface"
         * @param stop_interfaces command interfaces that should be stopped in the form "joint/interface"
         * @return hardware_interface::return_type
         */
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        /**
         * @brief Called when an error in any state or state transition is thrown.
         *
         * According to the [ros2_control documentation](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html#handling-of-errors-that-happen-during-read-and-write-calls):
         *
         * > Error handling follows the node lifecycle. If successful CallbackReturn::SUCCESS is returned and hardware is again in `UNCONFIGURED` state, if any ERROR or FAILURE happens the hardware ends in `FINALIZED` state and can not be recovered. The only option is to reload the complete plugin, but there is currently no service for this in the Controller Manager.
         *
         *
         * Since the hardware will immediatly return to the `unconfigured` state ([source](https://github.com/ros-controls/ros2_control/blob/d0836b7f12b89acb89bde83d4ba4308513b03204/hardware_interface/src/hardware_component.cpp#L247))
         * if the error could be handled we manually call the transition functions which would
         * normally be called to this state. Those are:
         * - <b>Previous state</b>: `active`
         *  - Deactivate hardware (on_deactivate()) -> `inactive`
         *  - Clean-Up hardware (on_cleanup()) -> `unconfigured`
         * - <b>Previous state</b>: `inactive`
         *  - Deactivate hardware (on_deactivate()) -> `inactive`
         *      - call the deactivate function anyway regardless if state was active or inactive.
         *        For example if the on_activate() function fails on Joint::enableStallguard()
         *        the joint will have been enabled, to disable it invoke on_deactivate().
         *  - Clean-Up hardware (on_cleanup()) -> `unconfigured`
         * .
         * In particular the deactivation is important. For example if a joint stalls the read() or write() methods throw an error,
         * which will be handled here and allow the hardware to be deactivated, disableing the joints to allow backdriving.
         *
         * @param previous_state
         * @return hardware_interface::CallbackReturn
         */
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
         * An unordered map is chosen to simplify acces via the joint name, as this conforms well with the ROS2_control hardware interface
         * The map does not need to be ordered. Search, insertion, and removal of elements have average constant-time complexity.
         *
         */
        std::unordered_map<std::string, Joint> _joints;

        /**
         * @brief unordered map storing the configuration struct of the joints.
         *
         * An unordered map is chosen to simplify acces via the joint name, as this conforms well with the ROS2_control hardware interface
         * The map does not need to be ordered. Search, insertion, and removal of elements have average constant-time complexity.
         *
         */
        std::unordered_map<std::string, joint_config_t> _joint_cfg;

        /**
         * @brief unordered map of sets storing the active command interfaces for each joint.
         * 
         * Each joint can have a set of active command interfaces. This type of structure is chosen to group interfaces by joint. 
         * In the write() function the interface name can simply be constructed by concatenating joint name with interface name.
         * Although currently only one active command interface is allowed at the time, a set can be used to store multiple command 
         * interfaces that are acceptable to be combined, for example it would be acceptable to set velocity 
         * and driver current and hence that would be an allowable combination.
         *
         * An unordered map is chosen to simplify acces via the joint name, as this conforms well with the ROS2_control hardware interface.
         * The map does not need to be ordered. Search, insertion, and removal of elements have average constant-time complexity. 
         *
         */
        std::unordered_map<std::string, std::set<std::string>> _joint_command_modes;



        int start_homing(const std::string name, float velocity);

        int stop_homing(const std::string name);
    };

} // namespace bioscara_hardware_interface

#endif // BIOSCARA_HARDWARE_INTERFACE_HPP_
