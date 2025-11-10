/* Author: sbstorz */

#include <QVBoxLayout>
#include <bioscara_rviz_plugin/bioscara_panel.hpp>
#include <rviz_common/display_context.hpp>

namespace bioscara_rviz_plugin
{
  BioscaraPanel::BioscaraPanel(QWidget *parent) : Panel(parent)
  {
    // Create a label and a button, displayed vertically (the V in VBox means vertical)
    const auto layout = new QVBoxLayout(this);
    label_ = new QLabel("[no data]");
    button_ = new QPushButton("GO!");
    _controllerList.set
    layout->addWidget(label_);
    layout->addWidget(button_);

    // Connect the event of when the button is released to our callback,
    // so pressing the button results in the callback being called.
    QObject::connect(button_, &QPushButton::released, this, &BioscaraPanel::buttonActivated);
  }

  BioscaraPanel::~BioscaraPanel() = default;

  void BioscaraPanel::onInitialize()
  {
    // Access the abstract ROS Node and
    // in the process lock it for exclusive use until the method is done.
    // node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    _node = std::make_shared<rclcpp::Node>("bioscara_rviz_panel");

    // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
    // (as per normal rclcpp code)
    // rclcpp::Node::SharedPtr node = _node;
    publisher_ = _node->create_publisher<std_msgs::msg::String>("/output", 10);
    subscription_ =
        _node->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
            "/controller_manager/activity", 10,
            std::bind(&BioscaraPanel::topicCallback, this, std::placeholders::_1));

    _list_controllers_client = _node->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
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

  // When the subscriber gets a message, this callback is triggered,
  // and then we copy its data into the widget's label
  void BioscaraPanel::topicCallback(
      const controller_manager_msgs::msg::ControllerManagerActivity &msg)
  {
    if (msg.controllers.size())
    {
      label_->setText(QString(msg.controllers[0].state.label.c_str()));
    }
  }

  // When the widget's button is pressed, this callback is triggered,
  // and then we publish a new message on our topic.
  void BioscaraPanel::buttonActivated()
  {
    auto message = std_msgs::msg::String();
    message.data = "Button clicked!";
    publisher_->publish(message);

    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

    auto result_future = _list_controllers_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(_node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(_node->get_logger(), "service call failed :(");
      _list_controllers_client->remove_pending_request(result_future);
    }
    auto result = result_future.get();
    RCLCPP_INFO(_node->get_logger(), "result of %s", result->controller[0].name.c_str());
  }

  void BioscaraPanel::timer_callback()
  {
    rclcpp::spin_some(_node);
  }

} // namespace bioscara_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bioscara_rviz_plugin::BioscaraPanel, rviz_common::Panel)
