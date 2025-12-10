/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: sbstorz */

#ifndef BIOSCARA_RVIZ_PANEL_HPP_
#define BIOSCARA_RVIZ_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <unordered_map>
#include <vector>

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/msg/named_lifecycle_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"

#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"

namespace Ui
{
  class BioscaraUI;
}

namespace bioscara_rviz_plugin
{
  using namespace controller_manager_msgs::msg;
  using namespace controller_manager_msgs::srv;

  class BioscaraPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit BioscaraPanel(QWidget *parent = 0);
    ~BioscaraPanel() override;

    void onInitialize() override;

  protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<ControllerManagerActivity>::SharedPtr cm_state_subsription_;
    rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<ConfigureController>::SharedPtr configure_controller_client_;
    rclcpp::Client<SetHardwareComponentState>::SharedPtr hardware_state_client_;

    /**
     * @brief Map correlating hardware component name to its lifecycle_msgs/State
     *
     * Stored as NamedLifecycleState to also simply access component name from map value.
     *
     */
    std::unordered_map<std::string, NamedLifecycleState> hardware_states_;

    /**
     * @brief Map correlating controller name to its lifecycle_msgs/State
     *
     * Stored as NamedLifecycleState to also simply access controller name from map value.
     */
    std::unordered_map<std::string, NamedLifecycleState> controller_states_;

    // QT elements
    Ui::BioscaraUI *ui_;
    QTimer *_timer;

    void cm_state_callback(const ControllerManagerActivity &msg);

    bool set_hardware_component_state(const std::string component, const lifecycle_msgs::msg::State target_state);

    bool configure_controller(const std::string controller);

    bool switch_controllers(const std::vector<std::string> &activate_controllers,
                            const std::vector<std::string> &deactivate_controllers,
                            const int32_t = SwitchController::Request::BEST_EFFORT,
                            const bool activate_asap = false,
                            const builtin_interfaces::msg::Duration timeout = builtin_interfaces::msg::Duration());

    bool set_controller_state(const std::string controller,
                              const lifecycle_msgs::msg::State target_state);

    void named_lcs_msg_to_map(const std::vector<NamedLifecycleState> &named_lcs_in,
                              std::unordered_map<std::string, NamedLifecycleState> &map_out);

    void print_cm_map(const std::string prefix,
                      const std::unordered_map<std::string, NamedLifecycleState> &map_in);

    /**
     * @brief Checks if the specified "must-have" keys are present in the state map.
     * If not a key is created with an empty State.
     *
     * This is done to avoid having to check for the existence of key when
     * accessing the map. Keys could be missing if an expected controller or
     * hardware component has not been loaded or has a different name.
     *
     * @param state_map
     * @param augment_vec
     */
    void augment_state_map(std::unordered_map<std::string, NamedLifecycleState> &state_map,
                           std::vector<std::string> augment_vec);
    /**
     * @brief Update all hardware and controller state lables.
     *
     */
    void update_labels_and_btns(void);

    void update_label_and_btn(
        const std::unordered_map<std::string, NamedLifecycleState> &state_map,
        QLabel *state_label,
        QPushButton *en_button,
        const std::string &state_key);

    void set_state_label(QLabel *label, const NamedLifecycleState &state);

    void set_en_btn(QPushButton *button, const NamedLifecycleState &state);

    /**
     * @brief Check under which conditions a component is allowed to be activated.
     *
     * Some controllers will only activate under certain conditions.
     * This functions disables the activation button if the coditions are met.
     *
     * @param component_key
     * @return true
     * @return false
     */
    bool check_activation_conditions(const std::string component_key);

    lifecycle_msgs::msg::State target_state_from_current(lifecycle_msgs::msg::State current_state);

  private Q_SLOTS:

    /**
     * @brief Tries to set the bioscara_arm hardware component
     *  to the `active` state and tries to enable the velocity_joint_trajectory_controller.
     *
     */
    void arm_en_btn_cb(void);

    /**
     * @brief Tries to set the bioscara_gripper_128 hardware component
     *  to the `active` state and tries to enable the gripper_controller.
     *
     */
    void gripper_en_btn_cb(void);

    /**
     * @brief Tries to set the velocity_joint_trajectory_controller state to `active`.
     *
     */
    void vjtc_ctrl_en_btn_cb(void);

    /**
     * @brief Tries to set the homing_controller state to `active`.
     *
     */
    void homing_ctrl_en_btn_cb(void);

    /**
     * @brief Tries to set the gripper_controller state to `active`.
     *
     */
    void gripper_ctrl_en_btn_cb(void);
    void timer_callback();
  };

} // namespace bioscara_rviz_plugin

#endif // BIOSCARA_RVIZ_PANEL_HPP_
