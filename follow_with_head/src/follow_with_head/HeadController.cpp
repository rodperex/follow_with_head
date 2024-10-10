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

HeadController::HeadController(const rclcpp::NodeOptions & options)
: Node("follow_controller", options),
  pan_pid_params_{0.0, 1.0, 0.0, 0.3},
  tilt_pid_params_{0.0, 1.0, 0.0, 0.3},
  pan_limit_(1.3),
  tilt_limit_(0.92),
  pan_joint_name_("pan_joint"),
  tilt_joint_name_("tilt_joint"),
  object_detected_(false)
{
  self_config();
}

void
HeadController::self_config()
{
  declare_parameter("pan_limit", pan_limit_);
  declare_parameter("tilt_limit", tilt_limit_);
  declare_parameter("pan_joint_name", pan_joint_name_);
  declare_parameter("tilt_joint_name", tilt_joint_name_);
  declare_parameter("pan_pid_min_ref", pan_pid_params_[0]);
  declare_parameter("pan_pid_max_ref", pan_pid_params_[1]);
  declare_parameter("pan_pid_min_output", pan_pid_params_[2]);
  declare_parameter("pan_pid_max_output", pan_pid_params_[3]);
  declare_parameter("tilt_pid_min_ref", tilt_pid_params_[0]);
  declare_parameter("tilt_pid_max_ref", tilt_pid_params_[1]);
  declare_parameter("tilt_pid_min_output", tilt_pid_params_[2]);
  declare_parameter("tilt_pid_max_output", tilt_pid_params_[3]);

  get_parameter("pan_joint_name", pan_joint_name_);
  get_parameter("tilt_joint_name", tilt_joint_name_);
  get_parameter("pan_limit", pan_limit_);
  get_parameter("tilt_limit", tilt_limit_);
  get_parameter("pan_joint_name", pan_joint_name_);
  get_parameter("tilt_joint_name", tilt_joint_name_);
  get_parameter("pan_pid_min_ref", pan_pid_params_[0]);
  get_parameter("pan_pid_max_ref", pan_pid_params_[1]);
  get_parameter("pan_pid_min_output", pan_pid_params_[2]);
  get_parameter("pan_pid_max_output", pan_pid_params_[3]);
  get_parameter("tilt_pid_min_ref", tilt_pid_params_[0]);
  get_parameter("tilt_pid_max_ref", tilt_pid_params_[1]);
  get_parameter("tilt_pid_min_output", tilt_pid_params_[2]);
  get_parameter("tilt_pid_max_output", tilt_pid_params_[3]);

  pan_pid_.set_pid(pan_pid_params_[0], pan_pid_params_[1], pan_pid_params_[2], pan_pid_params_[3]);
  tilt_pid_.set_pid(tilt_pid_params_[0], tilt_pid_params_[1], tilt_pid_params_[2],
      tilt_pid_params_[3]);

  RCLCPP_INFO(get_logger(), "Pan joint: %s", pan_joint_name_.c_str());
  RCLCPP_INFO(get_logger(), "Tilt joint: %s", tilt_joint_name_.c_str());

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&HeadController::joint_state_callback, this, _1));

  detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&HeadController::object_detection_callback, this, _1));

  follow_joint_trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    this, "/head_controller/follow_joint_trajectory");

  error_pub_ = create_publisher<error_msgs::msg::PanTiltError>("/error", 100);

  timer_ = create_wall_timer(100ms, std::bind(&HeadController::control_cycle, this));
}

void
HeadController::object_detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  object_x_angle_ = atan2(msg->detections[0].bbox.center.position.x,
      msg->detections[0].bbox.center.position.z);
  object_y_angle_ = atan2(msg->detections[0].bbox.center.position.y,
      msg->detections[0].bbox.center.position.z);

  RCLCPP_INFO(get_logger(), "OBJECT:  (%.2f, %.2f, %.2f) - [%.2f, %.2f]",
      msg->detections[0].bbox.center.position.x,
      msg->detections[0].bbox.center.position.y, msg->detections[0].bbox.center.position.z,
      object_x_angle_, object_y_angle_);

  last_detection_time_ = msg.get()->header.stamp;
  object_detected_ = true;
}

void
HeadController::joint_state_callback(
  sensor_msgs::msg::JointState::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Joint states received");
  last_state_ = std::move(msg);
}

void
HeadController::joint_trajectory_response_callback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
  goal_handle)
{
  (void) goal_handle;
  RCLCPP_DEBUG(get_logger(), "Goal accepted");
}

void
HeadController::joint_trajectory_feedback_callback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
  goal_handle,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  (void) goal_handle;
  RCLCPP_INFO(get_logger(), "Execution time: %.2d", feedback->desired.time_from_start.sec);
}

void
HeadController::joint_trajectory_result_callback(
  const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult
  & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(get_logger(), "La acción fue completada con éxito.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "La acción fue abortada.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "La acción fue cancelada.");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Resultado desconocido.");
      break;
  }
}

void
HeadController::send_joint_trajectory_goal(double pan, double tilt)
{
  if (!follow_joint_trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
  }

  auto goal_msg = FollowJointTrajectory::Goal();

  goal_msg.trajectory.joint_names.push_back(pan_joint_name_);
  goal_msg.trajectory.joint_names.push_back(tilt_joint_name_);

  goal_msg.trajectory.points.resize(1);
  goal_msg.trajectory.points[0].positions.resize(2);
  goal_msg.trajectory.points[0].velocities.resize(2);
  goal_msg.trajectory.points[0].accelerations.resize(2);
  goal_msg.trajectory.joint_names.resize(2);
  goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(0ms);

  goal_msg.trajectory.points[0].positions[0] = pan;
  goal_msg.trajectory.points[0].positions[1] = tilt;

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&HeadController::joint_trajectory_response_callback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&HeadController::joint_trajectory_feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&HeadController::joint_trajectory_result_callback, this,
    std::placeholders::_1);

  follow_joint_trajectory_client_->async_send_goal(goal_msg, send_goal_options);

}

void
HeadController::control_cycle()
{
  RCLCPP_DEBUG(get_logger(), "Control cycle");

  if (last_state_ == nullptr) {
    RCLCPP_WARN(get_logger(), "No joint state received yet");
    return;
  }

  if (!object_detected_) {
    RCLCPP_DEBUG(get_logger(), "No object detected. Stopping head");
    return;
  }

  error_msgs::msg::PanTiltError error_msg;
  double current_pan, current_tilt;

  for (size_t i = 0; i < last_state_->name.size(); i++) {
    if (last_state_->name[i] == pan_joint_name_) {
      current_pan = last_state_->position[i];
    } else if (last_state_->name[i] == tilt_joint_name_) {
      current_tilt = last_state_->position[i];
    }
  }

  // double command_pan = pan_pid_.get_output(object_x_angle_);
  // double command_tilt = tilt_pid_.get_output(object_y_angle_);
  double command_pan = object_x_angle_;
  double command_tilt = object_y_angle_;

  RCLCPP_INFO(get_logger(), "* COMMAND: [%.2f, %.2f]", current_pan - command_pan,
      current_tilt - command_tilt);

  send_joint_trajectory_goal(current_pan - command_pan, current_tilt - command_tilt);

  double pan_error = abs(object_x_angle_);
  double tilt_error = abs(object_y_angle_);

  error_msg.header.stamp = now();
  error_msg.pan_error = pan_error;
  error_msg.tilt_error = tilt_error;
  error_pub_->publish(error_msg);

  object_detected_ = false;

}

}  // namespace follow_with_head
