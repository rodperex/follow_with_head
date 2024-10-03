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

#include "follow_with_head/HeadController.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  bool use_ipc, real_time;

  if (argv[1] == std::string("True"))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using IPC");
    use_ipc = true;
  }
  else if (argv[1] == std::string("False"))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NOT Using IPC");
    use_ipc = false;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid argument. Usage: head_controller_main <use_ipc> <real_time>");
    return 1;
  }

  if (argv[2] == std::string("True"))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Real time");
    real_time = true;
  }
  else if (argv[2] == std::string("False"))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NOT Real time");
    real_time = false;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid argument. Usage: head_controller_main <use_ipc> <real_time>");
    return 1;
  }

  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  if (!real_time)
  {
    auto head_controller_node = std::make_shared<follow_with_head::HeadController>(rclcpp::NodeOptions().use_intra_process_comms(use_ipc));
    executor->add_node(head_controller_node);
    executor->spin();
  } else {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::unique_ptr<std::thread> thread = std::make_unique<std::thread>(
        [executor]() {
          sched_param sch;
          sch.sched_priority = 60;
          if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) == -1) {
            perror("pthread_setschedparam failed");
            exit(-1);
          }
          executor->spin();
        });
  }

  rclcpp::shutdown();
  return 0;
}
