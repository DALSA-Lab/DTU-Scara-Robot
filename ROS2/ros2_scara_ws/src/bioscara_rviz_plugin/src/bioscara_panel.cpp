/* Author: sbstorz */

#include <bioscara_rviz_plugin/bioscara_panel.hpp>
#include <rviz_common/display_context.hpp>

#include "ui_bioscara_rviz_plugin_frame.h"

namespace bioscara_rviz_plugin
{
  BioscaraPanel::BioscaraPanel(QWidget *parent) : Panel(parent)
  {
    ui_ = new Ui::BioscaraUI();
    ui_->setupUi(this);

    // Connect the event of when the button is released to our callback,
    // so pressing the button results in the callback being called.
    connect(ui_->arm_en_btn, SIGNAL(clicked()), this, SLOT(arm_en_btn_cb()));
    connect(ui_->gripper_en_btn, SIGNAL(clicked()), this, SLOT(gripper_en_btn_cb()));
    connect(ui_->vjtc_ctrl_en_btn, SIGNAL(clicked()), this, SLOT(vjtc_ctrl_en_btn_cb()));
    connect(ui_->homing_ctrl_en_btn, SIGNAL(clicked()), this, SLOT(homing_ctrl_en_btn_cb()));
    connect(ui_->gripper_ctrl_en_btn, SIGNAL(clicked()), this, SLOT(gripper_ctrl_en_btn_cb()));
  }

  BioscaraPanel::~BioscaraPanel() = default;

  void BioscaraPanel::onInitialize()
  {
    // Access the abstract ROS Node and
    // in the process lock it for exclusive use until the method is done.
    // node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    // TODO: Check this, might not need the qt timer then
    //     auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
    // if (!ros_node_abstraction)
    // {
    //   RCLCPP_INFO(logger_, "Unable to lock weak_ptr from DisplayContext in MotionPlanningFrame constructor");
    //   return;
    // }
    // node_ = ros_node_abstraction->get_raw_node();

    node_ = std::make_shared<rclcpp::Node>("bioscara_rviz_panel");

    cm_state_subsription_ =
        node_->create_subscription<ControllerManagerActivity>(
            "/controller_manager/activity", rclcpp::QoS(10).transient_local().reliable(),
            std::bind(&BioscaraPanel::cm_state_callback, this, std::placeholders::_1));

    hardware_state_client_ = node_->create_client<SetHardwareComponentState>(
        "/controller_manager/set_hardware_component_state");

    switch_controller_client_ = node_->create_client<SwitchController>(
        "/controller_manager/switch_controller");

    configure_controller_client_ = node_->create_client<ConfigureController>(
        "/controller_manager/configure_controller");
    // while (!_list_controllers_client->wait_for_service(std::chrono::seconds(1)))
    // {
    //   if (!rclcpp::ok())
    //   {
    //     RCLCPP_ERROR(node_->get_logger(), "client interrupted while waiting for service to appear.");
    //     return;
    //   }
    //   RCLCPP_INFO(node_->get_logger(), "waiting for service to appear...");
    // }

    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &BioscaraPanel::timer_callback);
    _timer->start(100);
  }

  void BioscaraPanel::cm_state_callback(const ControllerManagerActivity &msg)
  {
    named_lcs_msg_to_map(msg.controllers, controller_states_);
    named_lcs_msg_to_map(msg.hardware_components, hardware_states_);
    augment_state_map(controller_states_, {"velocity_joint_trajectory_controller",
                                           "gripper_controller",
                                           "homing_controller"});
    augment_state_map(hardware_states_, {"bioscara_arm",
                                         "bioscara_gripper_128"});

    print_cm_map("New controller states:", controller_states_);
    print_cm_map("New hardware states:", hardware_states_);

    update_labels_and_btns();
  }

  bool BioscaraPanel::set_hardware_component_state(const std::string component, const lifecycle_msgs::msg::State target_state)
  {
    auto req = std::make_shared<SetHardwareComponentState::Request>();
    req->name = component;
    req->target_state = target_state;

    auto result_future = hardware_state_client_->async_send_request(req);
    // Wait for the service response or timeout
    if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // Check if the response is valid
      if (!result_future.valid())
      {
        RCLCPP_WARN(node_->get_logger(), "Service call succeeded, but the response is invalid.");
        hardware_state_client_->remove_pending_request(result_future);
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed to complete within the timeout.");
      hardware_state_client_->remove_pending_request(result_future);
      return false;
    }

    auto result = result_future.get();
    RCLCPP_INFO(node_->get_logger(), "Called 'set_hardware_component_state': %s", result->ok ? "Success" : "Failure");
    return result->ok;
  }

  bool BioscaraPanel::configure_controller(const std::string controller)
  {
    auto req = std::make_shared<ConfigureController::Request>();
    req->name = controller;

    auto result_future = configure_controller_client_->async_send_request(req);
    // Wait for the service response or timeout
    if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // Check if the response is valid
      if (!result_future.valid())
      {
        RCLCPP_WARN(node_->get_logger(), "Service call succeeded, but the response is invalid.");
        configure_controller_client_->remove_pending_request(result_future);
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed to complete within the timeout.");
      configure_controller_client_->remove_pending_request(result_future);
      return false;
    }

    auto result = result_future.get();
    RCLCPP_INFO(node_->get_logger(), "Called 'configure_controller': %s", result->ok ? "Success" : "Failure");
    return result->ok;
  }

  bool BioscaraPanel::switch_controllers(const std::vector<std::string> &activate_controllers,
                                         const std::vector<std::string> &deactivate_controllers,
                                         const int32_t strictness,
                                         const bool activate_asap,
                                         const builtin_interfaces::msg::Duration timeout)
  {
    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers = activate_controllers;
    req->deactivate_controllers = deactivate_controllers;
    req->strictness = strictness;
    req->activate_asap = activate_asap;
    req->timeout = timeout;

    auto result_future = switch_controller_client_->async_send_request(req);
    // Wait for the service response or timeout
    if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // Check if the response is valid
      if (!result_future.valid())
      {
        RCLCPP_WARN(node_->get_logger(), "Service call succeeded, but the response is invalid.");
        switch_controller_client_->remove_pending_request(result_future);
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed to complete within the timeout.");
      switch_controller_client_->remove_pending_request(result_future);
      return false;
    }

    auto result = result_future.get();
    RCLCPP_INFO(node_->get_logger(), "Called 'switch_controllers': %s: '%s'", result->ok ? "Success" : "Failure", result->message.c_str());
    return result->ok;
  }

  bool BioscaraPanel::set_controller_state(const std::string controller,
                                           const lifecycle_msgs::msg::State target_state)
  {
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    switch (current_state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      switch (target_state.id)
      {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        return switch_controllers({}, {controller});
        break;

      default:
        break;
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      switch (target_state.id)
      {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        return switch_controllers({controller}, {});
        break;

      default:
        break;
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      switch (target_state.id)
      {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        if (!configure_controller(controller))
        {
          return false;
        }
        return switch_controllers({controller}, {});
        break;

      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        return configure_controller(controller);
        break;

      default:
        break;
      }
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Can not set controller state to target state, currently not supported.");
      break;
    }
    return false;
  }

  lifecycle_msgs::msg::State BioscaraPanel::target_state_from_current(lifecycle_msgs::msg::State current_state)
  {
    lifecycle_msgs::msg::State target_state;
    switch (current_state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      break;

    default:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
      break;
    }
    return target_state;
  }

  void BioscaraPanel::arm_en_btn_cb(void)
  {
    std::string component = "bioscara_arm";
    lifecycle_msgs::msg::State current_state = hardware_states_.at(component).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    if (!set_hardware_component_state(component, target_state))
    {
      return;
    }
    // TODO: activate vjtc if all joints are homed and activation will succeed.
    if (!set_controller_state("velocity_joint_trajectory_controller", target_state))
    {
      return;
    }
  }

  void BioscaraPanel::gripper_en_btn_cb(void)
  {
    std::string component = "bioscara_gripper_128";
    lifecycle_msgs::msg::State current_state = hardware_states_.at(component).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    if (!set_hardware_component_state(component, target_state))
    {
      return;
    }

    if (!set_controller_state("gripper_controller", target_state))
    {
      return;
    }
  }

  void BioscaraPanel::vjtc_ctrl_en_btn_cb(void)
  {
    std::string controller = "velocity_joint_trajectory_controller";
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    if (!set_controller_state(controller, target_state))
    {
      return;
    }
  }

  void BioscaraPanel::homing_ctrl_en_btn_cb(void)
  {
    std::string controller = "homing_controller";
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    if (!set_controller_state(controller, target_state))
    {
      return;
    }
  }

  void BioscaraPanel::gripper_ctrl_en_btn_cb(void)
  {
    std::string controller = "gripper_controller";
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    if (!set_controller_state(controller, target_state))
    {
      return;
    }
  }

  void BioscaraPanel::named_lcs_msg_to_map(const std::vector<NamedLifecycleState> &named_lcs_in,
                                           std::unordered_map<std::string, NamedLifecycleState> &map_out)
  {
    map_out.clear();
    for (const auto &lcs : named_lcs_in)
    {
      map_out.insert({lcs.name, lcs});
    }
  }

  void BioscaraPanel::print_cm_map(const std::string prefix,
                                   const std::unordered_map<std::string, NamedLifecycleState> &map_in)
  {
    std::string states = "";
    for (const auto &[name, state] : map_in)
    {
      states += (name + ":\t" + state.state.label + " (" + std::to_string(state.state.id) + ")\n");
    }
    RCLCPP_DEBUG(node_->get_logger(),
                 "%s\n%s", prefix.c_str(), states.c_str());
  }

  void BioscaraPanel::augment_state_map(std::unordered_map<std::string, NamedLifecycleState> &state_map,
                                        std::vector<std::string> augment_vec)
  {
    for (auto &key : augment_vec)
    {
      auto it = state_map.find(key);
      if (it == state_map.end())
      {
        NamedLifecycleState state;
        state.name = key;
        state_map.insert({key, state});
      }
    }
  }

  void BioscaraPanel::update_labels_and_btns(void)
  {
    update_label_and_btn(controller_states_, ui_->vjtc_ctrl_state_label,
                         ui_->vjtc_ctrl_en_btn, "velocity_joint_trajectory_controller");

    update_label_and_btn(controller_states_, ui_->homing_ctrl_state_label,
                         ui_->homing_ctrl_en_btn, "homing_controller");

    update_label_and_btn(controller_states_, ui_->gripper_ctrl_state_label,
                         ui_->gripper_ctrl_en_btn, "gripper_controller");

    update_label_and_btn(hardware_states_, ui_->arm_hardware_state_label,
                         ui_->arm_en_btn, "bioscara_arm");

    update_label_and_btn(hardware_states_, ui_->gripper_hardware_state_label,
                         ui_->gripper_en_btn, "bioscara_gripper_128");
  }

  void BioscaraPanel::update_label_and_btn(
      const std::unordered_map<std::string, NamedLifecycleState> &state_map,
      QLabel *state_label,
      QPushButton *en_button,
      const std::string &state_key)
  {

    set_state_label(state_label, state_map.at(state_key));
    set_en_btn(en_button, state_map.at(state_key));
  }

  void BioscaraPanel::set_state_label(QLabel *label, const NamedLifecycleState &state)
  {
    label->setText(QString(state.state.label.c_str()));
    switch (state.state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      label->setStyleSheet("QLabel { color : green; font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      label->setStyleSheet("QLabel { font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      label->setStyleSheet("QLabel { blue; font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      label->setStyleSheet("QLabel { color : red; font: bold }");
      break;

    default:
      label->setStyleSheet("QLabel {font: bold }");
      break;
    }
  }

  void BioscaraPanel::set_en_btn(QPushButton *button, const NamedLifecycleState &state)
  {
    std::string text;
    bool enable = true;
    switch (state.state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      button->setStyleSheet("QPushButton { color : red}");
      text = "Deactivate";
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      button->setStyleSheet("QPushButton { color : green}");
      text = "Activate";
      enable = check_activation_conditions(state.name);
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      button->setStyleSheet("QPushButton { color : orange}");
      text = "Configure";
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:

    default:
      text = "-";
      enable = false;
      break;
    }

    button->setText(QString(text.c_str()));
    button->setEnabled(enable);
  }

  bool BioscaraPanel::check_activation_conditions(const std::string component_key)
  {
    if (component_key == "velocity_joint_trajectory_controller")
    {
      if (hardware_states_.at("bioscara_arm").state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
      if (controller_states_.at("homing_controller").state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
    }
    if (component_key == "homing_controller")
    {
      if (hardware_states_.at("bioscara_arm").state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
      if (controller_states_.at("velocity_joint_trajectory_controller").state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
    }
    if (component_key == "gripper_controller")
    {
      if (hardware_states_.at("bioscara_gripper_128").state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
    }
    return true;
  }

  // When the widget's button is pressed, this callback is triggered,
  // and then we publish a new message on our topic.
  // void BioscaraPanel::buttonActivated()
  // {
  // RCLCPP_INFO(node_->get_logger(), "Clicked");
  // auto message = std_msgs::msg::String();
  // message.data = "Button clicked!";
  // publisher_->publish(message);

  // auto request = std::make_shared<ListControllers::Request>();

  // auto result_future = _list_controllers_client->async_send_request(request);
  // if (rclcpp::spin_until_future_complete(node_, result_future) !=
  //     rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(node_->get_logger(), "service call failed :(");
  //   _list_controllers_client->remove_pending_request(result_future);
  // }
  // auto result = result_future.get();
  // RCLCPP_INFO(node_->get_logger(), "result of %s", result->controller[0].name.c_str());
  // }

  void BioscaraPanel::timer_callback()
  {
    rclcpp::spin_some(node_);
  }

} // namespace bioscara_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bioscara_rviz_plugin::BioscaraPanel, rviz_common::Panel)
