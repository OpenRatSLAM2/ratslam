// Copyright (C) 2012 David Ball, Scott Heath, Michael Milford, Gordon Wyeth
//
// This file is part of openRatSLAM, ported for ROS 2.
//
// This node implements the main entry point for the View Template module.
// All code is written to be compatible with ROS 2 and follows ROS 2 C++ style guidelines.
// For code style, see ament_uncrustify, ament_cpplint, and ament_clang_format.

#include <iostream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#ifdef RATSLAM_ROS2_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <topological_msgs/msg/view_template.hpp>

#include "ratslam/local_view_match.h"
#include "ratslam/utils.h"

//#if HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
ratslam::LocalViewScene * lvs = nullptr;
bool use_graphics;
//#endif

using namespace ratslam;

class RatSLAMViewTemplate : public rclcpp::Node
{
public:
  RatSLAMViewTemplate()
  : Node("ratslam_view_template")
  {
    RCLCPP_INFO(this->get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
    RCLCPP_INFO(this->get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
    RCLCPP_INFO(
      this->get_logger(), "Distributed under the GNU GPL v3, see the included license file.");

    // Leitura dos parâmetros do ROS 2
    std::string topic_root;
    this->declare_parameter<std::string>("topic_root", "");
    this->get_parameter("topic_root", topic_root);

    double vt_min_patch_normalisation_std;
    int vt_patch_normalisation;
    double vt_normalisation;
    int vt_shift_match, vt_step_match;
    int vt_panoramic;
    double vt_match_threshold;
    bool vt_threshold_condition;
    int template_x_size, template_y_size;
    int image_crop_x_min, image_crop_x_max;
    int image_crop_y_min, image_crop_y_max;
    int vt_window_width, vt_window_height;
    bool enable;

    this->declare_parameter<double>("vt_min_patch_normalisation_std", 0.0);
    this->get_parameter("vt_min_patch_normalisation_std", vt_min_patch_normalisation_std);
    this->declare_parameter<int>("vt_patch_normalisation", 0);
    this->get_parameter("vt_patch_normalisation", vt_patch_normalisation);
    this->declare_parameter<double>("vt_normalisation", 0.0);
    this->get_parameter("vt_normalisation", vt_normalisation);
    this->declare_parameter<int>("vt_shift_match", 25);
    this->get_parameter("vt_shift_match", vt_shift_match);
    this->declare_parameter<int>("vt_step_match", 5);
    this->get_parameter("vt_step_match", vt_step_match);
    this->declare_parameter<int>("vt_panoramic", 0);
    this->get_parameter("vt_panoramic", vt_panoramic);
    this->declare_parameter<double>("vt_match_threshold", 0.03);
    this->get_parameter("vt_match_threshold", vt_match_threshold);
    this->declare_parameter<bool>("vt_threshold_condition", true);
    this->get_parameter("vt_threshold_condition", vt_threshold_condition);
    this->declare_parameter<int>("image_crop_x_min", 0);
    this->get_parameter("image_crop_x_min", image_crop_x_min);
    this->declare_parameter<int>("image_crop_x_max", -1);
    this->get_parameter("image_crop_x_max", image_crop_x_max);
    this->declare_parameter<int>("image_crop_y_min", 0);
    this->get_parameter("image_crop_y_min", image_crop_y_min);
    this->declare_parameter<int>("image_crop_y_max", -1);
    this->get_parameter("image_crop_y_max", image_crop_y_max);
    this->declare_parameter<int>("template_x_size", 1);
    this->get_parameter("template_x_size", template_x_size);
    this->declare_parameter<int>("template_y_size", 1);
    this->get_parameter("template_y_size", template_y_size);

    this->declare_parameter<bool>("enable", true);
    this->get_parameter("enable", enable);
    this->declare_parameter<int>("vt_window_width", 640);
    this->get_parameter("vt_window_width", vt_window_width);
    this->declare_parameter<int>("vt_window_height", 480);
    this->get_parameter("vt_window_height", vt_window_height);

    lv_ = std::make_unique<ratslam::LocalViewMatch>(
      vt_min_patch_normalisation_std, vt_patch_normalisation, vt_normalisation, vt_shift_match,
      vt_step_match, vt_panoramic, vt_match_threshold, vt_threshold_condition, template_x_size,
      template_y_size, image_crop_x_min, image_crop_x_max, image_crop_y_min, image_crop_y_max);

    pub_vt_ = this->create_publisher<topological_msgs::msg::ViewTemplate>(
      topic_root + "/view_template", 10);

    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_root + "/camera/image/compressed", 10,
      std::bind(&RatSLAMViewTemplate::image_callback, this, std::placeholders::_1));

    use_graphics = enable;
    if (use_graphics) {
      lvs = new ratslam::LocalViewScene(vt_window_width, vt_window_height, lv_.get());
    }
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::msg::Image::SharedPtr image_msg;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    image_msg = cv_ptr->toImageMsg();
    topological_msgs::msg::ViewTemplate vt_output;
    lv_->on_image(
      &image_msg->data[0], (image_msg->encoding == "bgr8" ? false : true), image_msg->width,
      image_msg->height);

    vt_output.header.stamp = this->get_clock()->now();
    vt_output.current_id = lv_->get_current_vt();
    vt_output.relative_rad = lv_->get_relative_rad();

    pub_vt_->publish(vt_output);

    //#ifdef HAVE_IRRLICHT
    if (use_graphics) {
      lvs->draw_all();
    }
    //#endif
  }

  std::unique_ptr<ratslam::LocalViewMatch> lv_;
  rclcpp::Publisher<topological_msgs::msg::ViewTemplate>::SharedPtr pub_vt_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RatSLAMViewTemplate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
