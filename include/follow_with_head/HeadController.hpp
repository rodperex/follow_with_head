// Copyright 2021 Intelligent Robotics Lab
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

#ifndef FOLLOW_WITH_HEAD__HEAD_CONTROLLER_HPP_
#define FOLLOW_WITH_HEAD__HEAD_CONTROLLER_HPP_

#include <memory>
#include <algorithm>
#include <utility>

#include "follow_with_head/PID.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"

namespace follow_with_head
{

class HeadController : public rclcpp::Node
{
public:
  HeadController();

  void control_cycle();

  void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);
  void object_detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

private:
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr error_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state_;

  double pan_pid_params_[4], tilt_pid_params_[4];

  PID pan_pid_, tilt_pid_;

  double object_x_angle_, object_y_angle_;

  double pan_limit_, tilt_limit_; // radians

};

}  // namespace follow_with_head

#endif  // FOLLOW_WITH_HEAD__HEAD_CONTROLLER_HPP_
