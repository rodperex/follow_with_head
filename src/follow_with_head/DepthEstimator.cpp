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

DepthEstimator::DepthEstimator()
: Node("depth_estimator_node")
{
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/input_depth", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  detection_sub_ =
    std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(
    this, "/detection_2d", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(100), *depth_sub_, *detection_sub_);
  sync_->registerCallback(std::bind(&DepthEstimator::callback_sync, this, _1, _2));

  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", 1, std::bind(&DepthEstimator::callback_info, this, _1));
  detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "/detection_3d", rclcpp::SensorDataQoS().reliable());
}

void
DepthEstimator::callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  RCLCPP_INFO(get_logger(), "Camera info received");

  model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  model_->fromCameraInfo(*msg);

  info_sub_ = nullptr;
}

void
DepthEstimator::callback_sync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg)
{
  if (model_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Camera Model not yet available");
    return;
  }

  if (image_msg->encoding != "16UC1" && image_msg->encoding != "32FC1") {
    RCLCPP_ERROR(get_logger(), "The image type has not depth info");
    return;
  }

  if (detection_pub_->get_subscription_count() > 0) {
    vision_msgs::msg::Detection3DArray detections_3d_msg;
    detections_3d_msg.header = detection_msg->header;

    cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(*image_msg, image_msg->encoding);

    for (const auto & detection : detection_msg->detections) {
      vision_msgs::msg::Detection3D detection_3d_msg;
      detection_3d_msg.results = detection.results;

      float depth;
      if (image_msg->encoding == "32FC1") {
        depth = depth_image_proc::DepthTraits<float>::toMeters(
        cv_depth_ptr->image.at<float>(
          cv::Point2d(detection.bbox.center.position.x, detection.bbox.center.position.y)));
      }

      if (image_msg->encoding == "16UC1") {
        depth = depth_image_proc::DepthTraits<uint16_t>::toMeters(
        cv_depth_ptr->image.at<uint16_t>(
          cv::Point2d(detection.bbox.center.position.x, detection.bbox.center.position.y)));
      }
      RCLCPP_INFO(get_logger(), "x: %.2f, y: %.2f, z: %.2f", detection.bbox.center.position.x,
          detection.bbox.center.position.y, depth);

      if (std::isnan(depth)) {
        continue;
      }

      cv::Point3d ray = model_->projectPixelTo3dRay(
        model_->rectifyPoint( // Rectifies taking into account the distortion model
          cv::Point2d(
            detection.bbox.center.position.x, detection.bbox.center.position.y)));

      ray = ray / ray.z; // Normalize so z is 1.0. Ray is in camera frame
      cv::Point3d point = ray * depth; // The point is in camera frame

      detection_3d_msg.bbox.center.position.x = point.x;
      detection_3d_msg.bbox.center.position.y = point.y;
      detection_3d_msg.bbox.center.position.z = point.z;

      detections_3d_msg.detections.push_back(detection_3d_msg);
    }

    if (!detections_3d_msg.detections.empty()) {
      detection_pub_->publish(detections_3d_msg);
    }
  }
}

}  // namespace follow_with_head
