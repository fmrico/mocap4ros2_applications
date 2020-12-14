// Copyright 2020 Intelligent Robotics Lab
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

#ifndef MOCAP_CONTROL__CONTROLLERNODE_HPP_
#define MOCAP_CONTROL__CONTROLLERNODE_HPP_

#include "mocap_msgs/msg/control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace mocap_control
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

  void start_system();
  void stop_system();
  void change_state();

private:
  void control_callback(const mocap_msgs::msg::Control::SharedPtr msg);

  rclcpp::Subscription<mocap_msgs::msg::Control>::SharedPtr control_sub_;
  rclcpp::Publisher<mocap_msgs::msg::Control>::SharedPtr control_pub_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

};

}  // namespace mocap_control

#endif  // MOCAP_CONTROL__CONTROLLERNODE_HPP_
