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

namespace follow_with_head
{

using std::placeholders::_1;

HSVFilter::HSVFilter()
: Node("hsv_filter")
{
  image_sub_ = image_transport::create_subscription(
    this, "/input_image", std::bind(&HSVFilter::image_callback, this, _1),
    "raw", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  image_pub_ = image_transport::create_publisher(
    this, "/output_image", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "detection_2d", rclcpp::SensorDataQoS().reliable());

  declare_parameter("min_h", h_);
  declare_parameter("min_s", s_);
  declare_parameter("min_v", v_);
  declare_parameter("max_h", H_);
  declare_parameter("max_s", S_);
  declare_parameter("max_v", V_);

  get_parameter("min_h", h_);
  get_parameter("min_s", s_);
  get_parameter("min_v", v_);
  get_parameter("max_h", H_);
  get_parameter("max_s", S_);
  get_parameter("max_v", V_);

  cv::namedWindow("Filtered Image");
  cv::createTrackbar("Low H", "Filtered Image", &h_, 360 / 2);
  cv::createTrackbar("High H", "Filtered Image", &H_, 360 / 2);
  cv::createTrackbar("Low S", "Filtered Image", &s_, 255);
  cv::createTrackbar("High S", "Filtered Image", &S_, 255);
  cv::createTrackbar("Low V", "Filtered Image", &v_, 255);
  cv::createTrackbar("High V", "Filtered Image", &V_, 255);
}

void
HSVFilter::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Convert sensor_msgs::msg::Image -> cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Change space color BGR -> HSV
  cv::Mat img_hsv;
  cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);

  cv::Mat1b filtered;
  cv::inRange(img_hsv, cv::Scalar(h_, s_, v_), cv::Scalar(H_, S_, V_), filtered);

  cv_bridge::CvImage out_image;
  out_image.header = msg->header;  // Same timestamp and tf frame as input image
  out_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;  // Or whatever
  cv_ptr->image.copyTo(out_image.image, filtered);

  cv::imshow("Filtered Image", out_image.image);

  if (image_pub_.getNumSubscribers() > 0) {
    image_pub_.publish(out_image.toImageMsg());
  }

  if (detection_pub_->get_subscription_count() > 0) {
    cv::Rect bbx = cv::boundingRect(filtered);

    auto m = cv::moments(filtered, true);
    if (m.m00 < 0.000001) {return;}
    int cx = m.m10 / m.m00;
    int cy = m.m01 / m.m00;

    vision_msgs::msg::Detection2D detection_msg;
    detection_msg.header = msg->header;
    detection_msg.bbox.center.position.x = cx;
    detection_msg.bbox.center.position.y = cy;
    detection_msg.bbox.size_x = bbx.width;
    detection_msg.bbox.size_y = bbx.height;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis_msg;
    hypothesis_msg.hypothesis.class_id = "colored_object";
    hypothesis_msg.hypothesis.score = 1.0;
    detection_msg.results.push_back(hypothesis_msg);

    vision_msgs::msg::Detection2DArray detection_array_msg;
    detection_array_msg.header = msg->header;
    detection_array_msg.detections.push_back(detection_msg);

    detection_pub_->publish(detection_array_msg);
  }

  cv::waitKey(30);
}

}  // namespace follow_with_head
