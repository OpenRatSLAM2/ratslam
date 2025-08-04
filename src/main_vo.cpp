// Copyright (C) 2012 David Ball, Scott Heath, Michael Milford, Gordon Wyeth
//
// This file is part of openRatSLAM, ported for ROS 2.
//
// This node implements the main entry point for the Visual Odometry module.
// All code is written to be compatible with ROS 2 and follows ROS 2 C++ style guidelines.
// For code style, see ament_uncrustify, ament_cpplint, and ament_clang_format.

#include <fstream>
#include <iostream>
using namespace std;

#include <boost/property_tree/ini_parser.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "ratslam/utils.h"
#include "ratslam/visual_odometry.h"
#ifdef RATSLAM_ROS2_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <image_transport/image_transport.hpp>

using namespace ratslam;

class RatSLAMVisualOdometry : public rclcpp::Node
{
public:
  RatSLAMVisualOdometry()
  : rclcpp::Node("ratslam_visual_odometry")
  {
    RCLCPP_INFO(get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
    RCLCPP_INFO(get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
    RCLCPP_INFO(get_logger(), "Distributed under the GNU GPL v3, see the included license file.");

    std::string topic_root;
    this->declare_parameter<std::string>("topic_root", "");
    this->get_parameter("topic_root", topic_root);

    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_root + "/camera/image/compressed", 10,
      std::bind(&RatSLAMVisualOdometry::image_callback, this, std::placeholders::_1));

    visual_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_root + "/odom", 1);

    counter_ = 0;

    int vtrans_image_x_min, vtrans_image_x_max, vtrans_image_y_min, vtrans_image_y_max;
    int vrot_image_x_min, vrot_image_x_max, vrot_image_y_min, vrot_image_y_max;
    double camera_fov_deg, camera_hz, vtrans_scaling, vtrans_max;

    this->declare_parameter<int>("vtrans_image_x_min", 0);
    this->get_parameter("vtrans_image_x_min", vtrans_image_x_min);
    this->declare_parameter<int>("vtrans_image_x_max", -1);
    this->get_parameter("vtrans_image_x_max", vtrans_image_x_max);
    this->declare_parameter<int>("vtrans_image_y_min", 0);
    this->get_parameter("vtrans_image_y_min", vtrans_image_y_min);
    this->declare_parameter<int>("vtrans_image_y_max", -1);
    this->get_parameter("vtrans_image_y_max", vtrans_image_y_max);
    this->declare_parameter<int>("vrot_image_x_min", 0);
    this->get_parameter("vrot_image_x_min", vrot_image_x_min);
    this->declare_parameter<int>("vrot_image_x_max", -1);
    this->get_parameter("vrot_image_x_max", vrot_image_x_max);
    this->declare_parameter<int>("vrot_image_y_min", 0);
    this->get_parameter("vrot_image_y_min", vrot_image_y_min);
    this->declare_parameter<int>("vrot_image_y_max", -1);
    this->get_parameter("vrot_image_y_max", vrot_image_y_max);
    this->declare_parameter<double>("camera_fov_deg", 50.0);
    this->get_parameter("camera_fov_deg", camera_fov_deg);
    this->declare_parameter<double>("camera_hz", 10.0);
    this->get_parameter("camera_hz", camera_hz);
    this->declare_parameter<double>("vtrans_scaling", 100.0);
    this->get_parameter("vtrans_scaling", vtrans_scaling);
    this->declare_parameter<double>("vtrans_max", 20.0);
    this->get_parameter("vtrans_max", vtrans_max);

    vo_ = new ratslam::VisualOdometry(
      vtrans_image_x_min, vtrans_image_x_max, vtrans_image_y_min, vtrans_image_y_max,
      vrot_image_x_min, vrot_image_x_max, vrot_image_y_min, vrot_image_y_max, camera_fov_deg,
      camera_hz, vtrans_scaling, vtrans_max);
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // msg (CompressedImage) does not have enconding neither width or height
    counter_++;
    // RCLCPP_INFO(this->get_logger(), "VO:image_callback{%d}", counter_);

    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::msg::Image::SharedPtr image_msg;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    image_msg = cv_ptr->toImageMsg();
    nav_msgs::msg::Odometry odom_output;

    vo_->on_image(
      &image_msg->data[0], (image_msg->encoding == "bgr8" ? false : true), image_msg->width,
      image_msg->height, &odom_output.twist.twist.linear.x, &odom_output.twist.twist.angular.z);

    odom_output.header.stamp = image_msg->header.stamp;

    visual_odom_pub_->publish(odom_output);
  }

  uint counter_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr visual_odom_pub_;
  ratslam::VisualOdometry * vo_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ratslam_visual_odometry = std::make_shared<RatSLAMVisualOdometry>();
  rclcpp::spin(ratslam_visual_odometry);
  rclcpp::shutdown();

  return 0;
}
