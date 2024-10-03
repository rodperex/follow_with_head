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
  bool use_ipc;

  if (argv[1] == std::string("True")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using IPC");
    use_ipc = true;
  } else if (argv[1] == std::string("False")) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NOT Using IPC");
    use_ipc = false;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Invalid argument. Usage: hsv_filter_main <use_ipc>");
    return 1;
  }

  rclcpp::init(argc, argv);

  auto filter_node =
    std::make_shared<follow_with_head::HSVFilter>(rclcpp::NodeOptions().use_intra_process_comms(
    use_ipc));

  rclcpp::spin(filter_node);

  rclcpp::shutdown();
  return 0;
}
