// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef SINGLE_TRIGGER_CONTROLLER__GPIO_COMMAND_CONTROLLER_HPP_
#define SINGLE_TRIGGER_CONTROLLER__GPIO_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "single_trigger_controller/single_trigger_controller_parameters.hpp"

namespace single_trigger_controller
{
  using CmdType = control_msgs::msg::DynamicInterfaceGroupValues;
  using StateType = control_msgs::msg::DynamicInterfaceGroupValues;
  using CallbackReturn = controller_interface::CallbackReturn;
  using InterfacesNames = std::vector<std::string>;
  using MapOfReferencesToCommandInterfaces = std::unordered_map<
      std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
  using MapOfReferencesToStateInterfaces =
      std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
  using StateInterfaces =
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

  /**
   * @brief
   * @todo Write documentation and link to GpioCommandController
   */
  class SingleTriggerController : public controller_interface::ControllerInterface
  {
  public:
    SingleTriggerController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief Realtime update method.
     * 
     * Only publishes the states.
     * 
     * @param time 
     * @param period 
     * @return controller_interface::return_type 
     */
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    void store_command_interface_types();
    void store_state_interface_types();

    /**
     * @brief Initializes the state message with NaN values for each configured state interface.
     *
     * {
     *  header:
     *  {
     *    stamp: 12345678
     *  },
     *  interface_groups: ['j1', 'gpio1'],
     *  interface_values:
     *  [
     *    {
     *      interface_names: ['home'],
     *      values: [NaN]
     *    },
     *    {
     *      interface_names:['ai0', 'ai1'],
     *      values:[NaN, NaN]
     *    }
     *  ]
     * }
     *
     */
    void initialize_state_msg();

    /**
     * @brief Publisher of state values.
     *
     * Loops over all interface groups and command interfaces
     * and calls the apply_state_value(). Then publishes the result using the realtime_state_publisher_
     *
     */
    void update_states();

    /**
     * @brief callback method for received command msg. Calls apply_command() for each command interface.
     *
     * @param commands
     * @return controller_interface::return_type
     */
    controller_interface::return_type update_commands(const CmdType &commands);
    template <typename T>
    std::unordered_map<std::string, std::reference_wrapper<T>> create_map_of_references_to_interfaces(
        const InterfacesNames &interfaces_from_params, std::vector<T> &configured_interfaces);
    template <typename T>
    bool check_if_configured_interfaces_matches_received(
        const InterfacesNames &interfaces_from_params, const T &configured_interfaces);

    /**
     * @brief retrieves the state values from the hardware and populates the state_msg with the result.
     *
     * calls the get_optional() of the method state_interfaces_map_ for the state referenced by the
     * indey and interface_index.
     *
     * @param state_msg
     * @param index
     * @param interface_index
     */
    void apply_state_value(
        StateType &state_msg, std::size_t index, std::size_t interface_index) const;

    /**
     * @brief calls the set_value() method for the hardwares command interface.
     *
     * @todo update this to set_optional()
     *
     * @param commands
     * @param index
     * @param command_interface_index
     */
    void apply_command(
        const CmdType &commands, std::size_t index,
        std::size_t command_interface_index) const;

    // bool should_broadcast_all_interfaces_of_configured_gpios() const;
    // void set_all_state_interfaces_of_configured_gpios();

    /**
     * @brief
     *
     * @param name
     * @return InterfacesNames
     */
    InterfacesNames get_state_interfaces_names(const std::string &name) const;

    /**
     * @brief Update the controller parameters.
     * 
     * Only called in on_configure() to update the parameters in case they have been changed.
     * 
     * @return true 
     */
    bool update_dynamic_map_parameters();
    // std::vector<hardware_interface::ComponentInfo> get_gpios_from_urdf() const;

  protected:
    /**
     * @brief vector of string holding all command interface names as configured from the parameters.
     *
     * Looks like this:
     * ['j1/home', 'gpio1/ao0', 'gpio1/ao1']
     *
     */
    InterfacesNames command_interface_types_;

    /**
     * @brief vector of string holding all state interface names as configured from the parameters.
     *
     * Looks like this:
     * ['j1/home', 'gpio1/ai0', 'gpio1/ai1']
     *
     */
    InterfacesNames state_interface_types_;

    /**
     * @brief map containing a reference to the actual hardwares command interfaces
     *
     */
    MapOfReferencesToCommandInterfaces command_interfaces_map_;

    /**
     * @brief map containing a reference to the actual hardwares state interfaces
     *
     */
    MapOfReferencesToStateInterfaces state_interfaces_map_;

    rclcpp::Subscription<CmdType>::SharedPtr command_subscriber_{};

    std::shared_ptr<rclcpp::Publisher<StateType>> state_publisher_{};
    std::shared_ptr<realtime_tools::RealtimePublisher<StateType>> realtime_state_publisher_{};
    StateType state_msg_;

    std::shared_ptr<single_trigger_controller_parameters::ParamListener> param_listener_{};
    single_trigger_controller_parameters::Params params_;

    /**
     * @brief Vector of all loaded hardware components from ros2_control urdf
     *
     */
    std::vector<hardware_interface::HardwareInfo> _hardware_components;
  };

} // namespace single_trigger_controller

#endif // SINGLE_TRIGGER_CONTROLLER__GPIO_COMMAND_CONTROLLER_HPP_
