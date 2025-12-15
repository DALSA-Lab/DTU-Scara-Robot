/* Author: sbstorz */

#ifndef BIOSCARA_RVIZ_PANEL_HPP_
#define BIOSCARA_RVIZ_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
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
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"

namespace Ui
{
  class BioscaraUI;
}

namespace bioscara_rviz_plugin
{
  using namespace controller_manager_msgs::msg;
  using namespace controller_manager_msgs::srv;
  using namespace control_msgs::msg;

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
    rclcpp::Subscription<DynamicJointState>::SharedPtr joint_state_subsription_;
    rclcpp::Publisher<DynamicInterfaceGroupValues>::SharedPtr homing_publisher_;
    rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<ConfigureController>::SharedPtr configure_controller_client_;
    rclcpp::Client<SetHardwareComponentState>::SharedPtr hardware_state_client_;

    rclcpp::TimerBase::SharedPtr prune_timer_;

    /**
     * @brief Map storing all joint state interface values by each joint.
     *
     */
    std::unordered_map<std::string, InterfaceValue> joint_states_;

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

    void cm_state_callback(const ControllerManagerActivity &msg);

    void joint_state_callback(const DynamicJointState &msg);

    /**
     * @brief Ensure the joint state broadcaster is active.
     * 
     * The JSB can become inactive, ensure that it comes active again.
     * 
     */
    void ensure_jsb_is_active(void);

    void set_hardware_component_state(const std::string component, const lifecycle_msgs::msg::State target_state);

    void configure_controller(const std::string controller);

    void switch_controllers(const std::vector<std::string> &activate_controllers,
                            const std::vector<std::string> &deactivate_controllers,
                            const int32_t = SwitchController::Request::BEST_EFFORT,
                            const bool activate_asap = false,
                            const builtin_interfaces::msg::Duration timeout = builtin_interfaces::msg::Duration());

    void set_controller_state(const std::string controller,
                              const lifecycle_msgs::msg::State target_state);

    void dynamic_joint_state_msg_to_map(const DynamicJointState &dynamic_joint_state_in,
                                        std::unordered_map<std::string, InterfaceValue> &map_out);

    void named_lcs_msg_to_map(const std::vector<NamedLifecycleState> &named_lcs_in,
                              std::unordered_map<std::string, NamedLifecycleState> &map_out);

    void print_cm_map(const std::string prefix,
                      const std::unordered_map<std::string, NamedLifecycleState> &map_in);

    /**
     * @brief Prepopulates a state map with a default NamedLifecycleState for
     * every specified key.
     *
     * This is done to avoid having to check for the existence of key when
     * accessing the map. Keys could be missing if hardware interfaces are
     * not specified or named differently.
     *
     * For every joint the state interfaces are initialized as follows:
     * - *position*: 0.0
     * - *velocity*: 0.0
     * - *home*: 0.0
     *
     * Some joints (the gripper) dont have all state interfaces, but they are
     * initialized regardless.
     *
     *
     * @param state_map The state map to prepopulate
     * @param augment_vec keys that are added/overwritten
     */
    void prepopulate_joint_state_map(std::unordered_map<std::string, InterfaceValue> &state_map,
                                     std::vector<std::string> augment_vec);

    /**
     * @brief Prepopulates a state map with a default NamedLifecycleState for
     * every specified key.
     *
     * This is done to avoid having to check for the existence of key when
     * accessing the map. Keys could be missing if an expected controller or
     * hardware component has not been loaded or has a different name.
     *
     *
     * @param state_map The state map to prepopulate
     * @param augment_vec keys that are added/overwritten
     */
    void prepopulate_state_map(std::unordered_map<std::string, NamedLifecycleState> &state_map,
                               std::vector<std::string> augment_vec);

    void update_homing_grp_state(void);

    void update_homing_state_labels(void);

    void set_homing_state_label(QLabel *label, const InterfaceValue &state);

    /**
     * @brief Update all hardware and controller state lables.
     *
     */
    void update_state_labels_and_btns(void);

    void update_state_label_and_btn(
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

    void homing_cmd(const std::string joint, const int cmd);


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

    void vjtc_ctrl_en_btn_cb(void);

    void homing_ctrl_en_btn_cb(void);

    void gripper_ctrl_en_btn_cb(void);

  };

} // namespace bioscara_rviz_plugin

#endif // BIOSCARA_RVIZ_PANEL_HPP_
