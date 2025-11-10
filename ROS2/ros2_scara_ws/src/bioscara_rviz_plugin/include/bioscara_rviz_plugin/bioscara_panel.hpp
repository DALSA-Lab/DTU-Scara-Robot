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

#ifndef RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_
#define RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "controller_manager_msgs/msg/named_lifecycle_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"

namespace bioscara_rviz_plugin
{
class BioscaraPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit BioscaraPanel(QWidget * parent = 0);
  ~BioscaraPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr subscription_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr _list_controllers_client;

  void topicCallback(const controller_manager_msgs::msg::ControllerManagerActivity& msg);

  QLabel * label_;
  QPushButton * button_;
  QTimer *_timer;

  rclcpp::Node::SharedPtr _node;

private Q_SLOTS:
  void buttonActivated();
  void timer_callback();
};

}  // namespace bioscara_rviz_plugin

#endif  // RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_
