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

#include <memory>

#include "follow_with_head/HSVFilter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  bool use_ipc_follow, use_ipc_hsv, use_ipc_depth;
  
  if (argv[1] == std::string("True")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using IPC");
    use_ipc_follow = true;
  } else if (argv[1] == std::string("False")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NOT Using IPC");
    use_ipc_follow = false;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Invalid argument. Usage: whole <use_ipc_follow> <use_ipc_hsv> <use_ipc_depth>");
    return 1;
  }

  if (argv[2] == std::string("True")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using IPC");
    use_ipc_hsv = true;
  } else if (argv[2] == std::string("False")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NOT Using IPC");
    use_ipc_hsv = false;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Invalid argument. Usage: whole <use_ipc_follow> <use_ipc_hsv> <use_ipc_depth>");
    return 1;
  }

  if (argv[3] == std::string("True")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using IPC");
    use_ipc_depth = true;
  } else if (argv[3] == std::string("False")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NOT Using IPC");
    use_ipc_depth = false;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Invalid argument. Usage: whole <use_ipc_follow> <use_ipc_hsv> <use_ipc_depth>");
    return 1;
  }

  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  auto head_controller_node =
    std::make_shared<follow_with_head::HSVFilter>(rclcpp::NodeOptions().use_intra_process_comms(
    use_ipc_follow));
  auto hsv_filter_node =
    std::make_shared<follow_with_head::HSVFilter>(rclcpp::NodeOptions().use_intra_process_comms(
    use_ipc_hsv));
  auto filter_node =
    std::make_shared<follow_with_head::HSVFilter>(rclcpp::NodeOptions().use_intra_process_comms(
    use_ipc_depth));

  executor->add_node(head_controller_node);
  executor->add_node(hsv_filter_node);
  executor->add_node(filter_node);

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
