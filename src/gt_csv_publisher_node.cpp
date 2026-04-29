#include "ros2qnukf/gt_csv_publisher_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ros2qnukf
{

GtCsvPublisherNode::GtCsvPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros2qnukf_gt_csv_publisher", options)
{
  path_gt_csv_ = this->declare_parameter<std::string>("path_gt_csv", path_gt_csv_);
  gt_pose_topic_ = this->declare_parameter<std::string>("gt_pose_topic", gt_pose_topic_);
  gt_path_topic_ = this->declare_parameter<std::string>("gt_path_topic", gt_path_topic_);
  frame_id_ = this->declare_parameter<std::string>("frame_id", frame_id_);
  publish_rate_hz_ = std::max(1.0, this->declare_parameter<double>("publish_rate_hz", publish_rate_hz_));
  publish_path_ = this->declare_parameter<bool>("publish_path", publish_path_);
  max_path_length_ = static_cast<int>(std::max<int64_t>(
      16, this->declare_parameter<int64_t>("max_path_length", max_path_length_)));

  if (path_gt_csv_.empty()) {
    throw std::runtime_error("path_gt_csv must be set for ros2qnukf_gt_csv_publisher");
  }

  load_csv(path_gt_csv_);

  gt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(gt_pose_topic_, 10);
  gt_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(gt_path_topic_, 2);
  gt_path_msg_.header.frame_id = frame_id_;

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GtCsvPublisherNode::timer_callback, this));

}

void GtCsvPublisherNode::load_csv(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open GT CSV: " + path);
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream stream(line);
    std::string field;
    std::vector<double> values;
    values.reserve(20);
    while (std::getline(stream, field, ',')) {
      if (field.empty()) {
        continue;
      }
      values.push_back(std::atof(field.c_str()));
    }

    // ETH/EuRoC format:
    // [timestamp(ns), p_x, p_y, p_z, q_w, q_x, q_y, q_z, ...]
    if (values.size() < 8) {
      continue;
    }

    GtSample sample{};
    sample.t_sec = 1e-9 * values[0];
    sample.position = Eigen::Vector3d(
      values[1], values[2], values[3]);
    sample.orientation = Eigen::Quaterniond(
      values[4], values[5], values[6], values[7]);
    sample.orientation.normalize();
    samples_.push_back(sample);
  }

  if (samples_.empty()) {
    throw std::runtime_error("No valid GT samples parsed from: " + path);
  }
}

GtCsvPublisherNode::GtSample GtCsvPublisherNode::interpolate(const double t_sec) const
{
  if (samples_.size() == 1U) {
    return samples_.front();
  }

  const auto later_it = std::lower_bound(
    samples_.begin(),
    samples_.end(),
    t_sec,
    [](const GtSample & sample, const double t) {return sample.t_sec < t;});

  if (later_it == samples_.begin()) {
    return samples_.front();
  }
  if (later_it == samples_.end()) {
    return samples_.back();
  }

  const auto earlier_it = std::prev(later_it);
  const double t0 = earlier_it->t_sec;
  const double t1 = later_it->t_sec;
  const double dt = t1 - t0;
  if (std::abs(dt) < 1e-9) {
    return *later_it;
  }

  const double alpha = (t_sec - t0) / dt;
  GtSample out{};
  out.t_sec = t_sec;
  out.position = earlier_it->position + alpha * (later_it->position - earlier_it->position);
  out.orientation = earlier_it->orientation.slerp(alpha, later_it->orientation);
  out.orientation.normalize();
  return out;
}

void GtCsvPublisherNode::timer_callback()
{
  if (!gt_pose_pub_) {
    return;
  }

  const auto now = this->now();
  const auto sample = interpolate(now.seconds());

  geometry_msgs::msg::PoseStamped pose{};
  pose.header.stamp = now;
  pose.header.frame_id = frame_id_;
  pose.pose.position.x = sample.position.x();
  pose.pose.position.y = sample.position.y();
  pose.pose.position.z = sample.position.z();
  pose.pose.orientation.w = sample.orientation.w();
  pose.pose.orientation.x = sample.orientation.x();
  pose.pose.orientation.y = sample.orientation.y();
  pose.pose.orientation.z = sample.orientation.z();
  gt_pose_pub_->publish(pose);

  if (publish_path_ && gt_path_pub_) {
    gt_path_msg_.header = pose.header;
    gt_path_msg_.poses.push_back(pose);
    if (static_cast<int>(gt_path_msg_.poses.size()) > max_path_length_) {
      gt_path_msg_.poses.erase(gt_path_msg_.poses.begin());
    }
    gt_path_pub_->publish(gt_path_msg_);
  }
}

}  // namespace ros2qnukf
