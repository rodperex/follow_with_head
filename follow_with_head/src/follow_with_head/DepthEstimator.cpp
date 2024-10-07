// Copyright 20243 Intelligent Robotics Lab
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


#include "follow_with_head/DepthEstimator.hpp"


namespace follow_with_head
{

using std::placeholders::_1;
using std::placeholders::_2;

DepthEstimator::DepthEstimator(const rclcpp::NodeOptions & options)
: Node("depth_estimator", options)
{
  self_config();
}

void
DepthEstimator::self_config()
{
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", 1, std::bind(&DepthEstimator::callback_info, this, _1));
  detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "/detection_3d", rclcpp::SensorDataQoS().reliable());

  // Create a timer to publish the image
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / 30),
    std::bind(&DepthEstimator::timerCallback, this));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/input_depth", 1, std::bind(&DepthEstimator::imageCallback, this, _1));

  detection2d_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detection_2d", 1, std::bind(&DepthEstimator::detectionCallback, this, _1));

  RCLCPP_INFO(get_logger(), "DepthEstimator initialized");
}

void
DepthEstimator::callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  RCLCPP_INFO(get_logger(), "Camera info received");

  model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  model_->fromCameraInfo(*msg);

  info_sub_ = nullptr;
}

void DepthEstimator::imageCallback(sensor_msgs::msg::Image::UniquePtr msg)
{
  last_image_ = std::move(msg);;
}

void DepthEstimator::detectionCallback(vision_msgs::msg::Detection2DArray::UniquePtr msg)
{
  last_detection_ = std::move(msg);
}

void DepthEstimator::timerCallback(){
  
  if (model_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Camera Model not yet available");
    return;
  }

  if (last_image_ == nullptr) {
    RCLCPP_WARN(get_logger(), "No image received");
    return;
  }

  if (last_detection_ == nullptr) {
    RCLCPP_WARN(get_logger(), "No detections received");
    return;
  }

  if (last_image_->encoding != "16UC1" && last_image_->encoding != "32FC1") {
    RCLCPP_ERROR(get_logger(), "The image type has not depth info");
    return;
  }

  if (detection_pub_->get_subscription_count() > 0) {
    vision_msgs::msg::Detection3DArray detections_3d_msg;
    detections_3d_msg.header = last_detection_->header;

    cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(*last_image_, last_image_->encoding);

    for (const auto & detection : last_detection_->detections) {
      vision_msgs::msg::Detection3D detection_3d_msg;
      detection_3d_msg.results = detection.results;
      cv::Point2d point2d(detection.bbox.center.position.x, detection.bbox.center.position.y);

      float depth;
      if (last_image_->encoding == "32FC1") {
        depth = depth_image_proc::DepthTraits<float>::toMeters(
          cv_depth_ptr->image.at<float>(point2d));
      }

      if (last_image_->encoding == "16UC1") {
        depth = depth_image_proc::DepthTraits<uint16_t>::toMeters(
          cv_depth_ptr->image.at<uint16_t>(point2d));
      }
      RCLCPP_DEBUG(get_logger(), "x: %.2f, y: %.2f, z: %.2f", detection.bbox.center.position.x,
          detection.bbox.center.position.y, depth);

      if (!std::isnan(depth) && depth > 0.0) {
        cv::Point3d ray = model_->projectPixelTo3dRay(
          model_->rectifyPoint(point2d)); // Rectifies taking into account the distortion model

        ray = ray / ray.z; // Normalize so z is 1.0. Ray is in camera frame
        cv::Point3d point = ray * depth; // The point is in camera frame

        RCLCPP_DEBUG(get_logger(), "x: %.2f, y: %.2f, z: %.2f", point.x, point.y, point.z);

        detection_3d_msg.bbox.center.position.x = point.x;
        detection_3d_msg.bbox.center.position.y = point.y;
        detection_3d_msg.bbox.center.position.z = point.z;

        detections_3d_msg.detections.push_back(detection_3d_msg);
      }
    }

    if (!detections_3d_msg.detections.empty()) {
      RCLCPP_DEBUG(get_logger(), "Publishing %lu detections", detections_3d_msg.detections.size());
      detection_pub_->publish(detections_3d_msg);
    } else {
      RCLCPP_DEBUG(get_logger(), "No valid detections");
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "No subscribers");
  }
}

}  // namespace follow_with_head
