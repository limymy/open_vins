// pose_to_file_ros2.cpp
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "utils/Recorder2.h"
#include "utils/print.h"

class PoseToFileNode : public rclcpp::Node {
public:
  PoseToFileNode() : Node("pose_to_file") {

    /* -------------------- Declare & read parameters -------------------- */
    this->declare_parameter<std::string>("verbosity", "INFO");
    this->declare_parameter<std::string>("topic", "/pose");
    this->declare_parameter<std::string>("topic_type", "PoseStamped");
    this->declare_parameter<std::string>("output", "poses.txt");

    std::string verbosity    = this->get_parameter("verbosity").as_string();
    std::string topic        = this->get_parameter("topic").as_string();
    std::string topic_type   = this->get_parameter("topic_type").as_string();
    std::string fileoutput   = this->get_parameter("output").as_string();

    ov_core::Printer::setPrintLevel(verbosity);

    PRINT_DEBUG("Done reading config values");
    PRINT_DEBUG(" - topic       = %s", topic.c_str());
    PRINT_DEBUG(" - topic_type  = %s", topic_type.c_str());
    PRINT_DEBUG(" - file        = %s", fileoutput.c_str());

    /* -------------------- Recorder object -------------------- */
    recorder_ = std::make_unique<ov_eval::Recorder>(fileoutput);

    /* -------------------- Create subscriber -------------------- */
    rclcpp::QoS qos( rclcpp::KeepLast(10) );   // 等价于 ROS1 中 queue_size=10

    if (topic_type == "PoseWithCovarianceStamped") {
      sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, qos,
          std::bind(&ov_eval::Recorder::callback_posecovariance, recorder_.get(), std::placeholders::_1));

    } else if (topic_type == "PoseStamped") {
      sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, qos,
          std::bind(&ov_eval::Recorder::callback_pose, recorder_.get(), std::placeholders::_1));

    } else if (topic_type == "TransformStamped") {
      sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
          topic, qos,
          std::bind(&ov_eval::Recorder::callback_transform, recorder_.get(), std::placeholders::_1));

    } else if (topic_type == "Odometry") {
      sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          topic, qos,
          std::bind(&ov_eval::Recorder::callback_odometry, recorder_.get(), std::placeholders::_1));

    } else {
      PRINT_ERROR("The specified topic type is not supported");
      PRINT_ERROR("topic_type = %s", topic_type.c_str());
      PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
      throw std::runtime_error("Unsupported topic type");
    }
  }

private:
  rclcpp::SubscriptionBase::SharedPtr sub_;
  std::unique_ptr<ov_eval::Recorder> recorder_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PoseToFileNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
