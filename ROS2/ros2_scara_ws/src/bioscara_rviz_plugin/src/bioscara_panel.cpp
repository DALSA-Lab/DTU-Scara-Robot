/* Author: sbstorz */

#include <QVBoxLayout>
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

    _node = std::make_shared<rclcpp::Node>("bioscara_rviz_panel");

    publisher_ = _node->create_publisher<std_msgs::msg::String>("/output", 10);
    cm_state_subsription_ =
        _node->create_subscription<ControllerManagerActivity>(
            "/controller_manager/activity", rclcpp::QoS(10).transient_local().reliable(),
            std::bind(&BioscaraPanel::cm_state_callback, this, std::placeholders::_1));

    hardware_state_client_ = _node->create_client<SetHardwareComponentState>(
        "/controller_manager/set_hardware_component_state");
    // while (!_list_controllers_client->wait_for_service(std::chrono::seconds(1)))
    // {
    //   if (!rclcpp::ok())
    //   {
    //     RCLCPP_ERROR(_node->get_logger(), "client interrupted while waiting for service to appear.");
    //     return;
    //   }
    //   RCLCPP_INFO(_node->get_logger(), "waiting for service to appear...");
    // }

    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &BioscaraPanel::timer_callback);
    _timer->start(100);
  }

  void BioscaraPanel::cm_state_callback(const ControllerManagerActivity &msg)
  {
    named_lcs_msg_to_map(msg.controllers, controller_states_);
    named_lcs_msg_to_map(msg.hardware_components, hardware_states_);
    print_cm_map("New controller states:", controller_states_);
    print_cm_map("New hardware states:", hardware_states_);
    update_state_labels();
  }

  void BioscaraPanel::arm_en_btn_cb(void)
  {
  }

  void BioscaraPanel::gripper_en_btn_cb(void)
  {
  }

  void BioscaraPanel::vjtc_ctrl_en_btn_cb(void)
  {
  }

  void BioscaraPanel::homing_ctrl_en_btn_cb(void)
  {
  }

  void BioscaraPanel::gripper_ctrl_en_btn_cb(void)
  {
  }

  void BioscaraPanel::named_lcs_msg_to_map(const std::vector<NamedLifecycleState> &named_lcs_in,
                                           std::unordered_map<std::string, lifecycle_msgs::msg::State> &map_out)
  {
    map_out.clear();
    for (const auto &lcs : named_lcs_in)
    {
      map_out.insert({lcs.name, lcs.state});
    }
  }

  void BioscaraPanel::print_cm_map(const std::string prefix,
                                   const std::unordered_map<std::string, lifecycle_msgs::msg::State> &map_in)
  {
    std::string states = "";
    for (const auto &[name, state] : map_in)
    {
      states += (name + ":\t" + state.label + " (" + std::to_string(state.id) + ")\n");
    }
    RCLCPP_INFO(_node->get_logger(),
                "%s\n%s", prefix.c_str(), states.c_str());
  }

  void BioscaraPanel::update_state_labels(void)
  {
    update_state_label(controller_states_, ui_->vjtc_ctrl_state_label, "velocity_joint_trajectory_controller");
    update_state_label(controller_states_, ui_->homing_ctrl_state_label, "homing_controller");
    update_state_label(controller_states_, ui_->gripper_ctrl_state_label, "gripper_controller");
    update_state_label(hardware_states_, ui_->arm_hardware_state_label, "bioscara_arm");
    update_state_label(hardware_states_, ui_->gripper_hardware_state_label, "bioscara_gripper_128");
  }

  void BioscaraPanel::update_state_label(
      const std::unordered_map<std::string, lifecycle_msgs::msg::State> &state_map,
      QLabel *label,
      const std::string &state_key)
  {

    auto it = state_map.find(state_key);
    if (it == state_map.end())
    {
      set_state_label(label, lifecycle_msgs::msg::State());
    }
    else
    {
      set_state_label(label, it->second);
    }
  }

  void BioscaraPanel::set_state_label(QLabel *label, const lifecycle_msgs::msg::State &state)
  {
    label->setText(QString(state.label.c_str()));
    switch (state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      label->setStyleSheet("QLabel { color : green; font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      label->setStyleSheet("QLabel { color : yellow; font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      label->setStyleSheet("QLabel { color : blue; font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      label->setStyleSheet("QLabel { color : red; font: bold }");
      break;

    default:
      label->setStyleSheet("QLabel {font: bold }");
      break;
    }
  }

  // When the widget's button is pressed, this callback is triggered,
  // and then we publish a new message on our topic.
  // void BioscaraPanel::buttonActivated()
  // {
  // RCLCPP_INFO(_node->get_logger(), "Clicked");
  // auto message = std_msgs::msg::String();
  // message.data = "Button clicked!";
  // publisher_->publish(message);

  // auto request = std::make_shared<ListControllers::Request>();

  // auto result_future = _list_controllers_client->async_send_request(request);
  // if (rclcpp::spin_until_future_complete(_node, result_future) !=
  //     rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(_node->get_logger(), "service call failed :(");
  //   _list_controllers_client->remove_pending_request(result_future);
  // }
  // auto result = result_future.get();
  // RCLCPP_INFO(_node->get_logger(), "result of %s", result->controller[0].name.c_str());
  // }

  void BioscaraPanel::timer_callback()
  {
    rclcpp::spin_some(_node);
  }

} // namespace bioscara_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bioscara_rviz_plugin::BioscaraPanel, rviz_common::Panel)
