#ifndef OV_EVAL_RECORDER2_H
#define OV_EVAL_RECORDER2_H

#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

/* ------- ROS 2 message & core headers ------- */
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace ov_eval {

/**
 * @brief This class takes in published poses and writes them to file.
 *
 * Output is a space-delimited text file compatible with downstream scripts.
 * If a covariance is present, the upper-triangular entries are also saved so
 * that NEES values can be computed.
 */
class Recorder {

public:
  /**
   * @brief Constructor: opens/creates the output file and prepares folders.
   * @param filename  Destination file path
   * @param logger    Optional rclcpp logger (defaults to "Recorder")
   */
  explicit Recorder(const std::string &filename,
                    const rclcpp::Logger &logger = rclcpp::get_logger("Recorder"))
      : logger_(logger) {

    /* ---- Create folder path (if needed) ---- */
    boost::filesystem::path dir(filename.c_str());
    if (boost::filesystem::create_directories(dir.parent_path())) {
      RCLCPP_INFO(logger_, "Created folder path to output file: %s",
                  dir.parent_path().c_str());
    }

    /* ---- Remove existing file ---- */
    if (boost::filesystem::exists(filename)) {
      RCLCPP_WARN(logger_, "Output file exists, deleting old file: %s",
                  filename.c_str());
      boost::filesystem::remove(filename);
    }

    /* ---- Open stream ---- */
    outfile.open(filename.c_str());
    if (outfile.fail()) {
      RCLCPP_ERROR(logger_, "Unable to open output file: %s", filename.c_str());
      std::exit(EXIT_FAILURE);
    }

    outfile << "# timestamp(s) tx ty tz qx qy qz qw "
               "Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 "
               "Pt11 Pt12 Pt13 Pt22 Pt23 Pt33"
            << std::endl;

    /* ---- Initialise members ---- */
    timestamp      = -1.0;
    q_ItoG         = Eigen::Vector4d(0, 0, 0, 1);
    p_IinG         = Eigen::Vector3d::Zero();
    cov_rot        = Eigen::Matrix<double, 3, 3>::Zero();
    cov_pos        = Eigen::Matrix<double, 3, 3>::Zero();
    has_covariance = false;
  }

  /* -------------------- Callbacks -------------------- */

  using OdomConstPtr   = nav_msgs::msg::Odometry::ConstSharedPtr;
  using PoseConstPtr   = geometry_msgs::msg::PoseStamped::ConstSharedPtr;
  using PoseCovConstPtr =
      geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr;
  using TfConstPtr     = geometry_msgs::msg::TransformStamped::ConstSharedPtr;

  void callback_odometry(const OdomConstPtr &msg) {
    timestamp = stampToSec(msg->header.stamp);
    q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
    p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z;

    /* covariance (XYZ / RPY ordering) */
    cov_pos << msg->pose.covariance[0],  msg->pose.covariance[1],
        msg->pose.covariance[2],  msg->pose.covariance[6],
        msg->pose.covariance[7],  msg->pose.covariance[8],
        msg->pose.covariance[12], msg->pose.covariance[13],
        msg->pose.covariance[14];

    cov_rot << msg->pose.covariance[21], msg->pose.covariance[22],
        msg->pose.covariance[23], msg->pose.covariance[27],
        msg->pose.covariance[28], msg->pose.covariance[29],
        msg->pose.covariance[33], msg->pose.covariance[34],
        msg->pose.covariance[35];

    has_covariance = true;
    write();
  }

  void callback_pose(const PoseConstPtr &msg) {
    timestamp = stampToSec(msg->header.stamp);
    q_ItoG << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    p_IinG << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    write();
  }

  void callback_posecovariance(const PoseCovConstPtr &msg) {
    timestamp = stampToSec(msg->header.stamp);
    q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
    p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z;

    cov_pos << msg->pose.covariance[0],  msg->pose.covariance[1],
        msg->pose.covariance[2],  msg->pose.covariance[6],
        msg->pose.covariance[7],  msg->pose.covariance[8],
        msg->pose.covariance[12], msg->pose.covariance[13],
        msg->pose.covariance[14];

    cov_rot << msg->pose.covariance[21], msg->pose.covariance[22],
        msg->pose.covariance[23], msg->pose.covariance[27],
        msg->pose.covariance[28], msg->pose.covariance[29],
        msg->pose.covariance[33], msg->pose.covariance[34],
        msg->pose.covariance[35];

    has_covariance = true;
    write();
  }

  void callback_transform(const TfConstPtr &msg) {
    timestamp = stampToSec(msg->header.stamp);
    q_ItoG << msg->transform.rotation.x, msg->transform.rotation.y,
        msg->transform.rotation.z, msg->transform.rotation.w;
    p_IinG << msg->transform.translation.x, msg->transform.translation.y,
        msg->transform.translation.z;
    write();
  }

protected:
  /* ------------ Helpers ------------ */
  static inline double stampToSec(const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) +
           static_cast<double>(t.nanosec) * 1e-9;
  }

  /**
   * @brief Write the current pose (and covariance if available) to file.
   */
  void write() {
    /* timestamp */
    outfile.precision(5);
    outfile.setf(std::ios::fixed, std::ios::floatfield);
    outfile << timestamp << " ";

    /* pose */
    outfile.precision(6);
    outfile << p_IinG.x() << " " << p_IinG.y() << " " << p_IinG.z() << " "
            << q_ItoG(0) << " " << q_ItoG(1) << " " << q_ItoG(2) << " "
            << q_ItoG(3);

    /* covariance (if present) */
    if (has_covariance) {
      outfile.precision(10);
      outfile << " " << cov_rot(0, 0) << " " << cov_rot(0, 1) << " "
              << cov_rot(0, 2) << " " << cov_rot(1, 1) << " "
              << cov_rot(1, 2) << " " << cov_rot(2, 2) << " "
              << cov_pos(0, 0) << " " << cov_pos(0, 1) << " "
              << cov_pos(0, 2) << " " << cov_pos(1, 1) << " "
              << cov_pos(1, 2) << " " << cov_pos(2, 2) << std::endl;
    } else {
      outfile << std::endl;
    }
  }

  /* ------------ Members ------------ */
  std::ofstream outfile;
  rclcpp::Logger logger_;

  bool   has_covariance = false;
  double timestamp;
  Eigen::Vector4d                q_ItoG;
  Eigen::Vector3d                p_IinG;
  Eigen::Matrix<double, 3, 3>    cov_rot;
  Eigen::Matrix<double, 3, 3>    cov_pos;
};

}  // namespace ov_eval

#endif  // OV_EVAL_RECORDER2_H
