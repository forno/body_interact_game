#ifndef BODY_POSE_RECEIVER_H
#define BODY_POSE_RECEIVER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>

class pose_receiver
{
  ros::Subscriber head_sub_;
  ros::Subscriber right_knee_sub_;
  ros::Subscriber left_knee_sub_;
  ros::Subscriber right_hand_sub_;
  ros::Subscriber left_hand_sub_;
  Eigen::Vector3d head_ {};
  Eigen::Vector3d right_knee_ {};
  Eigen::Vector3d left_knee_ {};
  Eigen::Vector3d right_hand_ {};
  Eigen::Vector3d left_hand_ {};

public:
  pose_receiver(ros::NodeHandle& n)
    : head_sub_ {n.subscribe("head", 1, &pose_receiver::head_cb, this)},
      right_knee_sub_ {n.subscribe("right_knee", 1, &pose_receiver::right_knee_cb, this)},
      left_knee_sub_ {n.subscribe("left_knee", 1, &pose_receiver::left_knee_cb, this)},
      right_hand_sub_ {n.subscribe("right_hand", 1, &pose_receiver::right_hand_cb, this)},
      left_hand_sub_ {n.subscribe("right_hand", 1, &pose_receiver::right_hand_cb, this)}
  {
  }

  Eigen::Vector3d get_head() const noexcept
  {
    return head_;
  }

  Eigen::Vector3d get_right_knee() const noexcept
  {
   return right_knee_;
  }

  Eigen::Vector3d get_left_knee() const noexcept
  {
    return left_knee_;
  }

  Eigen::Vector3d get_right_hand() const noexcept
  {
    return right_hand_;
  }

  Eigen::Vector3d get_left_hand() const noexcept
  {
    return left_hand_;
  }

private:
  void head_cb(const geometry_msgs::PointConstPtr& msg) noexcept
  {
    tf2::fromMsg(*msg, head_);
  }

  void right_knee_cb(const geometry_msgs::PointConstPtr& msg) noexcept
  {
    tf2::fromMsg(*msg, right_knee_);
  }

  void left_knee_cb(const geometry_msgs::PointConstPtr& msg) noexcept
  {
    tf2::fromMsg(*msg, left_knee_);
  }

  void right_hand_cb(const geometry_msgs::PointConstPtr& msg) noexcept
  {
    tf2::fromMsg(*msg, right_hand_);
  }

  void left_hand_cb(const geometry_msgs::PointConstPtr& msg) noexcept
  {
    tf2::fromMsg(*msg, left_hand_);
  }
};

#endif
