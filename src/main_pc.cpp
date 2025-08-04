// Copyright (C) 2012 David Ball, Scott Heath, Michael Milford, Gordon Wyeth
//
// This file is part of openRatSLAM, ported for ROS 2.
//
// This node implements the main entry point for the Pose Cell module.
// All code is written to be compatible with ROS 2 and follows ROS 2 C++ style guidelines.
// For code style, see ament_uncrustify, ament_cpplint, and ament_clang_format.

#include <boost/property_tree/ini_parser.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <topological_msgs/msg/topological_action.hpp>
#include <topological_msgs/msg/view_template.hpp>

#include "ratslam/posecell_network.h"
#include "ratslam/utils.h"

//#if HAVE_IRRLICHT
#include "graphics/posecell_scene.h"
ratslam::PosecellScene * pcs;
bool use_graphics;
//#endif

using namespace ratslam;

class RatSLAMPoseCells : public rclcpp::Node
{
public:
  RatSLAMPoseCells()
  : Node("ratslam_pose_cells")
  {
    RCLCPP_INFO(this->get_logger(), "openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
    RCLCPP_INFO(this->get_logger(), "RatSLAM algorithm by Michael Milford and Gordon Wyeth");
    RCLCPP_INFO(
      this->get_logger(), "Distributed under the GNU GPL v3, see the included license file.");

    std::string topic_root;
    this->declare_parameter<std::string>("topic_root", "");
    this->get_parameter("topic_root", topic_root);

    pub_pc_ = this->create_publisher<topological_msgs::msg::TopologicalAction>(
      topic_root + "/PoseCell/TopologicalAction", 10);

    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_root + "/odom", 10,
      std::bind(&RatSLAMPoseCells::odo_callback, this, std::placeholders::_1));

    sub_template_ = this->create_subscription<topological_msgs::msg::ViewTemplate>(
      topic_root + "/view_template", 10,
      std::bind(&RatSLAMPoseCells::template_callback, this, std::placeholders::_1));

    // Leitura dos parâmetros do ROS 2 para PosecellNetwork
    int pc_dim_xy, pc_dim_th, pc_w_e_dim, pc_w_i_dim;
    double pc_w_e_var, pc_w_i_var, pc_global_inhib;
    double vt_active_decay, pc_vt_inject_energy, pc_cell_x_size, exp_delta_pc_threshold,
      pc_vt_restore;
    bool enable;

    this->declare_parameter<int>("pc_dim_xy", 21);
    this->get_parameter("pc_dim_xy", pc_dim_xy);
    this->declare_parameter<int>("pc_dim_th", 36);
    this->get_parameter("pc_dim_th", pc_dim_th);
    this->declare_parameter<int>("pc_w_e_dim", 7);
    this->get_parameter("pc_w_e_dim", pc_w_e_dim);
    this->declare_parameter<int>("pc_w_i_dim", 5);
    this->get_parameter("pc_w_i_dim", pc_w_i_dim);
    this->declare_parameter<double>("pc_w_e_var", 1.0);
    this->get_parameter("pc_w_e_var", pc_w_e_var);
    this->declare_parameter<double>("pc_w_i_var", 2.0);
    this->get_parameter("pc_w_i_var", pc_w_i_var);
    this->declare_parameter<double>("pc_global_inhib", 0.00002);
    this->get_parameter("pc_global_inhib", pc_global_inhib);
    this->declare_parameter<double>("vt_active_decay", 1.0);
    this->get_parameter("vt_active_decay", vt_active_decay);
    this->declare_parameter<double>("pc_vt_inject_energy", 0.15);
    this->get_parameter("pc_vt_inject_energy", pc_vt_inject_energy);
    this->declare_parameter<double>("pc_cell_x_size", 1.0);
    this->get_parameter("pc_cell_x_size", pc_cell_x_size);
    this->declare_parameter<double>("exp_delta_pc_threshold", 2.0);
    this->get_parameter("exp_delta_pc_threshold", exp_delta_pc_threshold);
    this->declare_parameter<double>("pc_vt_restore", 0.05);
    this->get_parameter("pc_vt_restore", pc_vt_restore);
    this->declare_parameter<bool>("enable", true);
    this->get_parameter("enable", enable);

    // Passe os parâmetros para o construtor de PosecellNetwork
    pc_ = std::make_unique<ratslam::PosecellNetwork>(
      pc_dim_xy, pc_dim_th, pc_w_e_dim, pc_w_i_dim, pc_w_e_var, pc_w_i_var, pc_global_inhib,
      vt_active_decay, pc_vt_inject_energy, pc_cell_x_size, exp_delta_pc_threshold, pc_vt_restore);

    std::string media_path;
    this->declare_parameter<std::string>("media_path", "");
    this->get_parameter("media_path", media_path);
    std::string image_file;
    this->declare_parameter<std::string>("image_file", "");
    this->get_parameter("image_file", image_file);

    use_graphics = enable;
    if (use_graphics) {
      pcs = new ratslam::PosecellScene(pc_.get(), media_path, image_file);
    }
  }

private:
  void odo_callback(const nav_msgs::msg::Odometry::SharedPtr odo)
  {
    // auto start = std::chrono::high_resolution_clock::now();

    static rclcpp::Time prev_time(0, 0, RCL_ROS_TIME);

    if (prev_time.nanoseconds() > 0) {
      rclcpp::Time odo_time(odo->header.stamp.sec, odo->header.stamp.nanosec, RCL_ROS_TIME);
      double time_diff = (odo_time - prev_time).seconds();

      pc_output_.src_id = pc_->get_current_exp_id();
      pc_->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
      pc_output_.action = pc_->get_action();

      if (pc_output_.action != ratslam::PosecellNetwork::NO_ACTION) {
        pc_output_.header.stamp = this->get_clock()->now();
        pc_output_.dest_id = pc_->get_current_exp_id();
        pc_output_.relative_rad = pc_->get_relative_rad();
        pub_pc_->publish(pc_output_);
      }

      //#ifdef HAVE_IRRLICHT
      if (use_graphics) {
        pcs->update_scene();
        pcs->draw_all();
      }
      //#endif
    }

    prev_time = odo->header.stamp;

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> duration = end - start;
  }

  void template_callback(const topological_msgs::msg::ViewTemplate::SharedPtr vt)
  {
    // auto start = std::chrono::high_resolution_clock::now();

    pc_->on_view_template(vt->current_id, vt->relative_rad);

    //#ifdef HAVE_IRRLICHT
    if (use_graphics) {
      pcs->update_scene();
      pcs->draw_all();
    }
    //#endif

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> duration = end - start;
  }

  std::unique_ptr<ratslam::PosecellNetwork> pc_;
  rclcpp::Publisher<topological_msgs::msg::TopologicalAction>::SharedPtr pub_pc_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<topological_msgs::msg::ViewTemplate>::SharedPtr sub_template_;
  topological_msgs::msg::TopologicalAction pc_output_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RatSLAMPoseCells>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
