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
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "error_msgs/msg/pan_tilt_error.hpp"

#include "rclcpp/rclcpp.hpp"

namespace follow_with_head
{
using namespace std::chrono_literals;
class HeadController : public rclcpp::Node
{
public:
  HeadController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void control_cycle();

  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg);
  void object_detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

private:
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::Publisher<error_msgs::msg::PanTiltError>::SharedPtr error_pub_;
  void self_config();

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState::UniquePtr last_state_;

  double pan_pid_params_[4], tilt_pid_params_[4];

  PID pan_pid_, tilt_pid_;

  double object_x_angle_, object_y_angle_;

  double pan_limit_, tilt_limit_; // radians

  std::string pan_joint_name_, tilt_joint_name_;

  rclcpp::Time last_detection_time_;

  // const rclcpp::Duration MAX_DETECTION_AGE = rclcpp::Duration(5.0s);

  bool object_detected_;

};

}  // namespace follow_with_head

#endif  // FOLLOW_WITH_HEAD__HEAD_CONTROLLER_HPP_
