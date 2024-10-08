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

#ifndef FOLLOW_WITH_HEAD__DEPTHESTIMATOR_HPP_
#define FOLLOW_WITH_HEAD__DEPTHESTIMATOR_HPP_

#include <memory>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "image_geometry/pinhole_camera_model.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "depth_image_proc/depth_traits.hpp"
#include "opencv2/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

namespace follow_with_head
{

class DepthEstimator : public rclcpp::Node
{
public:
  DepthEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::shared_ptr<image_geometry::PinholeCameraModel> model_;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;

  rclcpp::TimerBase::SharedPtr timer_;  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection2d_sub_;

  sensor_msgs::msg::Image::UniquePtr last_image_;
  vision_msgs::msg::Detection2DArray::UniquePtr last_detection_;

  void callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg);
  void self_config();
  void timerCallback();
  void imageCallback(sensor_msgs::msg::Image::UniquePtr msg);
  void detectionCallback(vision_msgs::msg::Detection2DArray::UniquePtr msg);
};

}  // namespace follow_with_head

#endif  // FOLLOW_WITH_HEAD__DEPTHESTIMATOR_HPP_
