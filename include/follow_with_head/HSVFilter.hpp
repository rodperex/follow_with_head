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

#ifndef FOLLOW_WITH_HEAD__HSVFILTER_HPP_
#define FOLLOW_WITH_HEAD__HSVFILTER_HPP_

#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "rclcpp/rclcpp.hpp"

namespace follow_with_head
{

class HSVFilter : public rclcpp::Node
{
public:
  HSVFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void self_config();

  int h_, s_, v_;
  int H_, S_, V_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
};

}  // namespace follow_with_head

#endif  // FOLLOW_WITH_HEAD__HSVFILTER_HPP_
