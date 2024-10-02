// Copyright 2024 Intelligent Robotics Lab
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


#include "follow_with_head/HeadController.hpp"


namespace follow_with_head
{

using std::placeholders::_1;
using namespace std::chrono_literals;

HeadController::HeadController()
: Node("head_controller"),
  pan_pid_params_{0.0, 1.0, 0.0, 0.3},
  tilt_pid_params_{0.0, 1.0, 0.0, 0.3},
  pan_limit_(1.3),
  tilt_limit_(0.92)
{

  declare_parameter("pan_limit", pan_limit_);
  declare_parameter("tilt_limit", tilt_limit_);

  declare_parameter("pan_pid_min_ref", pan_pid_params_[0]);
  declare_parameter("pan_pid_max_ref", pan_pid_params_[1]);
  declare_parameter("pan_pid_min_output", pan_pid_params_[2]);
  declare_parameter("pan_pid_max_output", pan_pid_params_[3]);
  pan_pid_.set_pid(pan_pid_params_[0], pan_pid_params_[1], pan_pid_params_[2], pan_pid_params_[3]);

  declare_parameter("tilt_pid_min_ref", tilt_pid_params_[0]);
  declare_parameter("tilt_pid_max_ref", tilt_pid_params_[1]);
  declare_parameter("tilt_pid_min_output", tilt_pid_params_[2]);
  declare_parameter("tilt_pid_max_output", tilt_pid_params_[3]);
  tilt_pid_.set_pid(tilt_pid_params_[0], tilt_pid_params_[1], tilt_pid_params_[2],
      tilt_pid_params_[3]);

  joint_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&HeadController::joint_state_callback, this, _1));

  detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&HeadController::object_detection_callback, this, _1));

  joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_command", 100);

  error_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/error", 100);

  timer_ = create_wall_timer(100ms, std::bind(&HeadController::control_cycle, this));
}

void
HeadController::object_detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  object_x_angle_ = atan2(msg->detections[0].bbox.center.position.x,
      msg->detections[0].bbox.center.position.z);
  object_y_angle_ = atan2(msg->detections[0].bbox.center.position.y,
      msg->detections[0].bbox.center.position.z);

  RCLCPP_INFO(get_logger(), "Object detected at:\n - Pan: %.2f rad\n - Tilt: %.2f rad",
      object_x_angle_, object_y_angle_);
}

void
HeadController::joint_state_callback(
  control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg)
{
  RCLCPP_INFO(get_logger(), "Joint state received");
  last_state_ = std::move(msg);
}

void
HeadController::control_cycle()
{
  RCLCPP_INFO(get_logger(), "Control cycle");

  if (last_state_ == nullptr) {
    RCLCPP_WARN(get_logger(), "No joint state received yet");
    return;
  }

  trajectory_msgs::msg::JointTrajectory command_msg;
  std_msgs::msg::Float32MultiArray error_msg;

  command_msg.header.stamp = now();

  command_msg.joint_names = last_state_->joint_names;

  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].time_from_start = rclcpp::Duration(0ms);

  double current_pan = last_state_->feedback.positions[0];
  double current_tilt = last_state_->feedback.positions[1];

  RCLCPP_INFO(get_logger(), "Current pan: %.2f rad\n - Current tilt: %.2f rad", current_pan,
      current_tilt);

  double desired_pan = pan_pid_.get_output(object_x_angle_);
  double desired_tilt = tilt_pid_.get_output(object_y_angle_);

  command_msg.points[0].positions[0] = std::clamp(desired_pan, -pan_limit_,
      pan_limit_);
  command_msg.points[0].positions[1] = std::clamp(desired_tilt, -tilt_limit_,
      tilt_limit_);

  joint_pub_->publish(command_msg);

  double pan_error = object_x_angle_ - current_pan;
  double tilt_error = object_y_angle_ - current_tilt;

  RCLCPP_INFO(get_logger(), "Pan error: %.2f rad\n - Tilt error: %.2f rad", pan_error, tilt_error);

  error_msg.data.push_back(pan_error);
  error_msg.data.push_back(tilt_error);
  error_pub_->publish(error_msg);
}

}  // namespace follow_with_head
